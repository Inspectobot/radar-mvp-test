from matplotlib import pylab as plt


plt_colors = ['b','g','r','c','m','y','k']


def plot_time_domain(t,data,title="Plot of voltage vs time",figure_number=1,t_range=-1,output="screen"):
    plt.figure(figure_number)
    for i,x in enumerate(data):
        plt.plot(t/1e-6,x,plt_colors[i])
    plt.xlabel("Time in us")
    plt.ylabel("Amplitude in volts")
    plt.title(title)
    if output == "screen":
        plt.show()
    else:
        pass


