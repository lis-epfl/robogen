import pylab
import numpy as np
import sys

if __name__ == "__main__":
    if len(sys.argv) < 2 :
        print "Usage: python plot_results.py BestAvgStd.txt"
        exit()

    results_file = sys.argv[1]
    results = np.loadtxt(results_file)
    


    best_plot, = pylab.plot(results[:,1], '-', color='red')
    pylab.hold(True)
    mean_plot, = pylab.plot(results[:,2], '-', color='black')
    pylab.legend([best_plot, mean_plot],['Best', 'Avg'], loc='best')
    ax = pylab.gca()
    ax.set_ylabel('Fitness',size=20)
    ax.set_xlabel('Generation',size=20)
    pylab.show()
    
