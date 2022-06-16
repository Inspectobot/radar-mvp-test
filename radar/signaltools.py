import cupy
import cupyx

import cupyx.scipy.signal as signal
from cupy._core import internal
from cupy._core import _scalar

from .arraytools import *

def lfilter(b, a, x, axis=-1, zi=None):
    if a.ndim != 1 or b.ndim != 1:
        raise ValueError('object of too small depth for desired array')
    if x.ndim == 0:
        raise ValueError('x must be at least 1-D')
    axis = internal._normalize_axis_index(axis, x.ndim)
    if a[0] == 0:
        raise ValueError("a[0] cannot be 0")
    dtype = cupy.result_type(b, a, x)
    n_filt = max(a.size, b.size)
    n_samples = x.shape[axis]
    n_signals = x.size // n_samples
    z_shape = axis_replace(x.shape, axis, n_filt - 1)
    if zi is not None:
        # do not broadcast zi, but do expand singleton dims
        if zi.ndim != x.ndim:
            raise ValueError('object of too small depth for desired array')
        # check the trivial case where zi is the right shape first
        if zi.shape != z_shape:
            zi_strides = list(zi.strides)
            for k in range(x.ndim):
                if k != axis and zi.shape[k] == 1 and z_shape[k] != 1:
                    zi_strides[k] = 0
                elif zi.shape[k] != z_shape[k]:
                    raise ValueError(
                        'Unexpected shape for zi: expected {}, found {}'.
                        format(z_shape, zi.shape))
            zi = cupy.lib.stride_tricks.as_strided(zi, z_shape, zi_strides)
        dtype = cupy.result_type(dtype, zi)

    # Normalize arrays
    needs_scaling = a[0] != 1
    b = b.astype(dtype, order='C', copy=needs_scaling)
    a = a.astype(dtype, order='C', copy=needs_scaling)
    if needs_scaling:
        b /= a[0]
        a /= a[0]
    x = x.astype(dtype, copy=False)

    # Apply the numerator coefficients
    if zi is not None and b.size < a.size:
        b = cupy.pad(b, (0, a.size-b.size))
    # TODO: eventually the next line should be method='auto'
    # TODO: you need to use https://github.com/cupy/cupy/pull/3748 (soon to be merged as of Aug 13 2020)
    out_full = signal.convolve(x, axis_broadcast(b, x.ndim, axis), method='direct')
    if zi is not None:
        axis_slice(out_full, stop=n_filt-1, axis=axis)[...] += zi

    if len(a) == 1:
        # No denominator
        if zi is None:
            return axis_slice(out_full, stop=n_samples, axis=axis)
        out = axis_slice(out_full, stop=n_samples, axis=axis)
        zf = axis_slice(out_full, start=n_samples, axis=axis)
        return out, zf

    # Apply the denominator coefficients
    int_dtype = cupy.int32 if x.size < (1 << 31) else cupy.int64
    kernel = _get_lfilter_kernel(
        axis, x.ndim, a.size, zi is not None,
        _scalar.get_typename(dtype),
        _scalar.get_typename(int_dtype))
    info = cupy.array((n_signals, n_samples) + out_full.shape, dtype=int_dtype)
    kernel(((n_signals+128-1)//128,), (128,), (a, out_full, info))
    if zi is None:
        return axis_slice(out_full, stop=n_samples, axis=axis)
    out = axis_slice(out_full, stop=n_samples, axis=axis)
    zf = axis_slice(out_full, start=n_samples, axis=axis)
    return out, zf


_FILTER_GENERAL = '''
#include "cupy/carray.cuh"
#include <cupy/complex.cuh>
typedef unsigned char byte;
typedef {val_type} T;
typedef {int_type} idx_t;

template <typename T>
__device__ T* row(T* ptr, idx_t i, idx_t axis, idx_t ndim, const idx_t* shape) {{
    idx_t index = 0, stride = 1;
    for (idx_t a = ndim; --a > 0; ) {{
        if (a != axis) {{
            index += (i % shape[a]) * stride;
            i /= shape[a];
        }}
        stride *= shape[a];
    }}
    return ptr + index + stride * i;
}}
'''


@cupy.memoize(for_each_device=True)
def _get_lfilter_kernel(axis, ndim, n_filt, compute_zf, val_type, int_type):
    if compute_zf:
        zf_computation = '''
            for (idx_t n = n_samples; n < n_samples + {n_filt} - 1; ++n) {{
                T accuml = 0;
                for (idx_t f = n - n_samples + 1; f < stop; f++) {{
                    accuml += a[f]*y_i[-f*y_elem_stride];
                }}
                y_i[0] -= accuml;
                y_i += y_elem_stride;
            }}
        '''.format(n_filt=n_filt)
    else:
        zf_computation = ''

    return cupy.RawKernel(
        _FILTER_GENERAL.format(val_type=val_type, int_type=int_type)+'''
extern "C" __global__
void cupyx_lfilter(const T* __restrict__ a, T* __restrict__ y,
                   idx_t* __restrict__ info) {{
    const idx_t n_signals = info[0], n_samples = info[1],
        * __restrict__ shape = info+2;

    const idx_t stop = min({n_filt}, n_samples);
    idx_t y_elem_stride = 1;
    for (int a = {axis}; ++a < {ndim}; ) {{ y_elem_stride *= shape[a]; }}

    CUPY_FOR (i, n_signals) {{
        T* __restrict__ y_i = row(y, i, {axis}, {ndim}, shape);
        y_i += y_elem_stride;
        for (idx_t n = 1; n < stop; ++n) {{
            T accuml = 0;
            for (idx_t f = 1; f <= n; f++) {{
                accuml += a[f]*y_i[-f*y_elem_stride];
            }}
            y_i[0] -= accuml;
            y_i += y_elem_stride;
        }}
        for (idx_t n = {n_filt}; n < n_samples; ++n) {{
            T accuml = 0;
            for (idx_t f = 1; f < {n_filt}; f++) {{
                accuml += a[f]*y_i[-f*y_elem_stride];
            }}
            y_i[0] -= accuml;
            y_i += y_elem_stride;
        }}
        {zf_comp}
    }}
}}'''.format(axis=axis, ndim=ndim, n_filt=n_filt, zf_comp=zf_computation),
        'cupyx_lfilter')

def lfilter_zi(b, a):
    """Construct initial conditions for lfilter for step response steady-state.

    Compute an initial state ``zi`` for the ``lfilter`` function that
    corresponds to the steady state of the step response.

    A typical use of this function is to set the initial state so that the
    output of the filter starts at the same value as the first element of
    the signal to be filtered.

    Args:
        b, a (cupy.ndarray): The IIR filter coefficients. See ``lfilter`` for
            more information.

    Returns:
        cupy.ndarray: The initial state for the filter.

    .. seealso:: :func:`scipy.signal.lfilter_zi`
    .. seealso:: :func:`cupyx.scipy.signal.lfilter`
    .. seealso:: :func:`cupyx.scipy.signal.lfiltic`
    """
    if b.ndim != 1:
        raise ValueError("Numerator b must be 1-D.")
    if a.ndim != 1:
        raise ValueError("Denominator a must be 1-D.")

    a = cupy.trim_zeros(a, 'f')

    if a.size < 2:
        raise ValueError("Must have at least two coefficients after removing "
                         "leading 0s")

    if a[0] != 1.0:
        # Normalize the coefficients so a[0] == 1.
        b = b / a[0]
        a = a / a[0]

    # Pad a or b with zeros so they are the same length.
    n = max(len(a), len(b))
    if len(a) < n:
        a = cupy.r_[a, cupy.zeros(n - len(a))]
    elif len(b) < n:
        b = cupy.r_[b, cupy.zeros(n - len(b))]

    # Solve zi = A*zi + B
    A = a[1:].copy()
    A[0] += 1
    B = b[1:] - a[1:] * b[0]
    zi = cupy.empty(n - 1)
    zi[0] = zi_0 = B.sum() / A.sum()
    zi_ = A[:-1].cumsum(out=zi[1:])
    zi_ *= zi_0
    zi_ -= B[:-1].cumsum(out=B[:-1])
    return zi


def filtfilt(b, a, x, axis=-1, padtype='odd', padlen=None, method='pad',
             irlen=None):
    """Apply a digital filter forward and backward to a signal.

    This function applies a linear digital filter twice, once forward and once
    backwards. The combined filter has zero phase and a filter order twice that
    of the original.

    The function provides options for handling the edges of the signal. The
    function ``sosfiltfilt`` (and filter design using ``output='sos'``) should
    be preferred over ``filtfilt`` for most filtering tasks, as second-order
    sections have fewer numerical problems.

    Args:
        b (cupy.ndarray): The numerator coefficient vector of the filter.
        a (cupy.ndarray): The denominator coefficient vector of the filter. If
            ``a[0]`` is not 1, then ``a`` and ``b`` are normalized by ``a[0]``.
        x (cupy.ndarray): The array of data to be filtered.
        axis (int, optional): The axis of ``x`` to which the filter is applied.
            Default is -1.
        padtype (str or None, optional): Must be 'odd', 'even', 'constant', or
            None. This determines the type of extension to use for the padded
            signal to which the filter is applied. If ``padtype`` is None, no
            padding is used. The default is 'odd'.
        padlen (int or None, optional): The number of elements by which to
            extend ``x`` at both ends of ``axis`` before applying the filter.
            This value must be less than ``x.shape[axis] - 1``. ``padlen=0``
            implies no padding. The default value is ``3*max(len(a), len(b))``.
        method (str, optional): Determines the method for handling the edges of
            the signal, either "pad" or "gust". When "pad", the signal is
            padded; the type of padding is determined by ``padtype`` and
            ``padlen``, and ``irlen`` is ignored. When "gust", Gustafsson's
            method is used, and ``padtype`` and ``padlen`` are ignored.
        irlen (int or None, optional): When ``method`` is "gust", ``irlen``
            specifies the length of the impulse response of the filter. If
            ``irlen`` is None, no part of the impulse response is ignored. For
            a long signal, specifying ``irlen`` can significantly improve the
            performance of the filter.

    Returns:
        cupy.ndarray: The filtered output with the same shape as ``x``.

    .. seealso:: :func:`scipy.signal.sosfilt`
    .. seealso:: :func:`scipy.signal.lfilter`
    .. seealso:: :func:`scipy.signal.lfilter_zi`
    .. seealso:: :func:`scipy.signal.lfiltic`
    .. seealso:: :func:`scipy.signal.savgol_filter`
    .. seealso:: :func:`cupyx.scipy.signal.filtfilt`
    """
    if method not in ("pad", "gust"):
        raise ValueError("method must be 'pad' or 'gust'.")
    axis = internal._normalize_axis_index(axis, x.ndim)

    if method == "gust":
        raise NotImplementedError("method='gust' not implemented yet")
        # y, z1, z2 = _filtfilt_gust(b, a, x, axis=axis, irlen=irlen)
        # return y

    # method == "pad"
    edge, ext = _validate_pad(padtype, padlen, x, axis,
                              ntaps=max(len(a), len(b)))

    # Get the steady state of the filter's step response.
    zi = lfilter_zi(b, a)

    # Reshape zi and create x0 so that zi*x0 broadcasts to the correct value
    # for the 'zi' keyword argument to lfilter
    zi = axis_broadcast(zi, x.ndim, axis=axis)
    x0 = axis_slice(ext, stop=1, axis=axis)

    # Forward filter
    y, _ = lfilter(b, a, ext, axis=axis, zi=zi * x0)

    # Backward filter
    # Create y0 so zi*y0 broadcasts appropriately
    y0 = axis_slice(y, start=-1, axis=axis)
    y, _ = lfilter(b, a, axis_reverse(y, axis=axis), axis=axis, zi=zi*y0)

    # Reverse y
    y = axis_reverse(y, axis=axis)

    if edge > 0:
        # Slice the actual signal from the extended signal
        y = axis_slice(y, start=edge, stop=-edge, axis=axis)

    return y


def _validate_pad(padtype, padlen, x, axis, ntaps):
    if padtype not in ('even', 'odd', 'constant', None):
        raise ValueError(("Unknown value '%s' given to padtype.  padtype "
                          "must be 'even', 'odd', 'constant', or None.") %
                         padtype)

    if padtype is None:
        padlen = 0

    edge = ntaps*3 if padlen is None else padlen

    # x's 'axis' dimension must be bigger than edge.
    if x.shape[axis] <= edge:
        raise ValueError("The length of the input vector x must be greater "
                         "than padlen, which is %d." % edge)

    if padtype is not None and edge > 0:
        # Make an extension of length edge at each end of the input array.
        if padtype == 'even':
            ext = even_ext(x, edge, axis=axis)
        elif padtype == 'odd':
            ext = odd_ext(x, edge, axis=axis)
        else:
            ext = const_ext(x, edge, axis=axis)
    else:
        ext = x
    return edge, ext
