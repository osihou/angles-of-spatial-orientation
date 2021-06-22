import matplotlib.pyplot as plt
import numpy as np
import csv

from numpy.core.fromnumeric import size
import errors 

def scatter_hist(x, y, ax, ax_histx, ax_histy, binwidth, params):

    ax_histx.tick_params(axis="x", labelbottom=False)
    ax_histy.tick_params(axis="y", labelleft=False)

    mean_x = np.mean(x)
    mean_y = np.mean(y)

    std_x = np.std(x)
    std_y = np.std(y)

    print(np.cov([x,y]))

    ax.scatter(x, y, s=0.1)
    

    binwidth = binwidth
    xymax = max(np.max(np.abs(x)), np.max(np.abs(y)))
    lim = (int(xymax/binwidth) + 1) * binwidth

    bins = np.arange(-lim, lim + binwidth, binwidth)
    
    ax_histx.hist(x, bins=bins)
    ax_histx.axvline(linewidth=1, color='black')
    

    ax_histy.hist(y, bins=bins, orientation='horizontal')
    ax_histy.axhline(linewidth=1, color='black')

    ax.axhline(linewidth=1, color='black')
    ax.axvline(linewidth=1, color='black')

    ax_histy.axhline(y=mean_y, linewidth=1, color='#d62728')
    ax_histx.axvline(x=mean_x, linewidth=1, color='#d62728')

    ax_histx.set_title(r'$\mu_x=%.5f$, $\sigma_x=%.5f$' % (mean_x, std_x))
    ax_histy.set_title(r'$\mu_y=%.5f$, $\sigma_y=%.5f$' % (mean_y, std_y))
    
    ax.scatter(mean_x, mean_y, c='red', s=50, marker='x')

    ax.set_xlabel(params[2])
    ax.set_ylabel(params[3])


def plot_analyse(inpath, binwidth, params):
    file_reader = csv.reader(open(inpath) , delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    lsterrorX = []
    lsterrorY = []
    for row in file_reader:
        errorX = float(row[params[0]])
        errorY = float(row[params[1]])

        lsterrorX.append(errorX)
        lsterrorY.append(errorY)



    left, width = 0.1, 0.65
    bottom, height = 0.1, 0.65
    spacing = 0.005


    rect_scatter = [left, bottom, width, height]
    rect_histx = [left, bottom + height + spacing, width, 0.2]
    rect_histy = [left + width + spacing, bottom, 0.2, height]


    fig = plt.figure(figsize=(10, 10))
    plt.style.use('bmh')

    ax = fig.add_axes(rect_scatter)
    ax_histx = fig.add_axes(rect_histx, sharex=ax)
    ax_histy = fig.add_axes(rect_histy, sharey=ax)

    scatter_hist(lsterrorX, lsterrorY, ax, ax_histx, ax_histy, binwidth, params)

    plt.show()

def normal_chart_2_time(inpath, Y1, Y2, Y1t, Y2t, params):

    file_reader = csv.reader(open(inpath) , delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    lstX = []
    lstY = []
    linecount = 0
    time = []
    for row in file_reader:
        linecount += 1

        errorX = float(row[Y1])
        errorY = float(row[Y2])

        lstX.append(errorX)
        lstY.append(errorY + errors.cape_error2(errorX, errorY) * params[0])

        time.append(linecount)

    plt.style.use('bmh')
    plt.figure(figsize = (10,10))

    plt.plot( time, lstX, linestyle='-',color='black', markerfacecolor='black', marker='.', markeredgecolor="black", markersize=5)
    plt.plot( time, lstY, linestyle='-',color='red', markerfacecolor='red', marker='.', markeredgecolor="red", markersize=5)

    plt.legend([Y1t,Y2t], loc = 1)
    plt.xlabel("TIME[s]")
    plt.ylabel(params[4])

    plt.ylim(params[1],params[2])
    plt.xlim(0,linecount)

    plt.title(params[5])

    plt.show()

def normal_chart_1_time(inpath, Y1, Y1t,  params):
    file_reader = csv.reader(open(inpath) , delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    lstX = []
    linecount = 0
    time = []
    for row in file_reader:
        linecount += 1

        errorX = float(row[Y1])

        lstX.append(errorX*params[0])

        time.append(linecount)

    plt.style.use('bmh')
    plt.figure(figsize = (10,10))

    plt.plot( time, lstX, linestyle='-',color='black', markerfacecolor='black', marker='.', markeredgecolor="black", markersize=5)

    plt.legend([Y1t], loc = 1)
    plt.xlabel("TIME[s]")
    plt.ylabel(params[4])

    plt.ylim(params[1],params[2])
    plt.xlim(0,linecount)

    plt.title(params[5])

    plt.show()

def smoll(x):
    if x < -170:
        return x + 360

    return x



def normal_chart_XY(inpath, Y1, Y2, Y1t, params):
    file_reader = csv.reader(open(inpath) , delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    lstX = []
    lstY = []

    for row in file_reader:

        errorX = float(row[Y1])
        errorY = float(row[Y2])

        lstX.append(errorX)
        lstY.append(errorY)


    plt.style.use('bmh')
    plt.figure(figsize = (10,10))

    plt.plot( lstX, lstY, linestyle='-',color='black', markerfacecolor='black', marker='.', markeredgecolor="black", markersize=1, linewidth=1)

    plt.legend([Y1t], loc = 1)
    plt.xlabel(params[0])
    plt.ylabel(params[1])

    plt.ylim(params[2],params[3])
    #plt.xlim(-0.1,0.1)

    plt.axhline(y=np.mean(lstY), linewidth=1, color='#d62728')

    print(np.mean(lstY))

    plt.title(params[4])

    plt.show()

def normal_chart_XYZ(inpath, Y1, Y2, Y1t,Y2t, params):
    file_reader = csv.reader(open(inpath) , delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    lstX = []
    lstY = []
    lstZ = []

    for row in file_reader:

        errorX = float(row[Y1])
        errorY = float(row[Y2])
        errorZ = float(row[Y2])

        lstX.append(errorX)
        lstY.append(errorY)
        lstZ.append(errorZ)


    plt.style.use('bmh')
    plt.figure(figsize = (10,10))

    plt.plot( lstX, lstY, linestyle='-',color='black', markerfacecolor='black', marker='.', markeredgecolor="black", markersize=1, linewidth=1)
    plt.plot( lstX, lstZ, linestyle='-',color='red', markerfacecolor='red', marker='.', markeredgecolor="red", markersize=1, linewidth=1)

    plt.legend([Y1t,Y2t], loc = 1)
    plt.xlabel(params[0])
    plt.ylabel(params[1])

    plt.ylim(params[2],params[3])
    #plt.xlim(-0.1,0.1)

    plt.axhline(y=np.mean(lstY), linewidth=1, color='#d62728')

    print(np.mean(lstY))

    plt.title(params[4])

    plt.show()