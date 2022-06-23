import cupy


def axis_replace(shape, axis, value):
    if axis < 0:
        axis += len(shape)
    return shape[:axis] + (value,) + shape[axis+1:]


def axis_seq(axis, ndim, value, default):
    if axis < 0:
        axis += ndim
    default = (default,)
    return default*axis + (value,) + default*(ndim-axis-1)


def axis_broadcast(a, ndim, axis=-1):
    return a[axis_seq(axis, ndim, slice(None), None)]
    # or:
    # return cupy.broadcast_to(a, axis_seq(axis, ndim, a.size, 1))


def axis_slice(a, start=None, stop=None, step=None, axis=-1):
    # See scipy.signal._arraytools.axis_slice for documentation
    return a[axis_seq(axis, a.ndim, slice(start, stop, step), slice(None))]


def axis_reverse(a, axis=-1):
    # See scipy.signal._arraytools.axis_reverse for documentation
    return axis_slice(a, step=-1, axis=axis)


def odd_ext(x, n, axis=-1):
    # See scipy.signal._arraytools.odd_ext for documentation
    if n < 1:
        return x
    if n > x.shape[axis] - 1:
        raise ValueError(("The extension length n (%d) is too big. " +
                         "It must not exceed x.shape[axis]-1, which is %d.")
                         % (n, x.shape[axis] - 1))
    L = 2*axis_slice(x, stop=1, axis=axis)
    R = 2*axis_slice(x, start=-1, axis=axis)
    left_ext = L - axis_slice(x, start=n, stop=0, step=-1, axis=axis)
    right_ext = R - axis_slice(x, start=-2, stop=-n-2, step=-1, axis=axis)
    return cupy.concatenate((left_ext, x, right_ext), axis=axis)


def even_ext(x, n, axis=-1):
    # See scipy.signal._arraytools.even_ext for documentation
    if n < 1:
        return x
    if n > x.shape[axis] - 1:
        raise ValueError(("The extension length n (%d) is too big. " +
                         "It must not exceed x.shape[axis]-1, which is %d.")
                         % (n, x.shape[axis] - 1))
    return cupy.pad(x, axis_seq(axis, x.ndim, (n, n), (0, 0)), 'reflect')


def const_ext(x, n, axis=-1):
    # See scipy.signal._arraytools.const_ext for documentation
    if n < 1:
        return x
    return cupy.pad(x, axis_seq(axis, x.ndim, (n, n), (0, 0)), 'edge')


def zero_ext(x, n, axis=-1):
    # See scipy.signal._arraytools.zero_ext for documentation
    if n < 1:
        return x
    return cupy.pad(x, axis_seq(axis, x.ndim, (n, n), (0, 0)), 'constant')
