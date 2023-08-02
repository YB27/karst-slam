import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import scipy.stats
from matplotlib import rc

"""
Functions for plotting the threshold graph
It is the Figure 9 in [1]

References
----------
.. [1] Breux, Yohan, and Lionel Lapierre. 
        "Elevation Angle Estimations of Wide-Beam Acoustic Sonar Measurements 
        for Autonomous Underwater Karst Exploration." Sensors 20.14 (2020): 4028.
"""

# Different global parameters for the plotting 
params = {
        'axes.labelsize': 15,
        'font.size': 20,
        'legend.fontsize': 10,
        'xtick.labelsize': 15,
        'ytick.labelsize': 15,
        'text.usetex': True,
        'figure.figsize': [9, 7],
        }
plt.rcParams.update(params)

def plotThresholdGraph():
        """ 
                Plot of the threshold (see eq 46 to eq 52) 
        """ 

        plt.figure(0)

        b = np.deg2rad(35)
        sigma_m_sqr = b**2/(4*9)
        x = np.linspace(-0.5*b, 0.5*b, num=500)
        x_abs = np.abs(x)
        y = (b + 2*x_abs)**2 *(b - 2*x_abs)/(4*(3*b + 2*x_abs))
        y_min = np.minimum(y, np.full((len(y),), sigma_m_sqr))


        plt.gca().spines['right'].set_visible(False)
        plt.gca().spines['top'].set_visible(False)
        plt.xlabel(r'$\bar{\theta}$ (rad)')
        plt.ylabel(r'$Variance$')
        plt.plot(x, y, label=r'$\sigma^2_l$',lw=2.5,color='c', ls='--')
        plt.plot(x, y, label=r'$\sigma^2_l$',lw=2.5,color='c', ls='--')
        plt.axhline(y=sigma_m_sqr, color='r', lw=2.5, label=r'$\sigma^2_{m}(k=3)$', ls='--')
        plt.plot(x, y_min,lw=2.5, label=r'$\sigma^2_{T}$', color= 'g')
        plt.legend()
        plt.savefig("threshold_graph.pdf",  bbox_inches='tight')

def plotBetaDefinedByMode():
        """ 
                Plot of the beta parameter (of a Beta distribution) from the distribution mode 
                Just used for testing.
        """ 
        
        # plt.figure(0)
        b = np.deg2rad(35)
        sigma_theta_sqr = 0.001

        nMode = 10
        mode = np.linspace(-0.5*b + 1e-3, 0.5*b - 1e-3, num = nMode

        theta_x = np.arange(-0.5*b, 0.5*b, 0.001)
        x = np.linspace(-100, 100, num=5000)
        a = (b + 2.*mode)/(b - 2.*mode)
        c = -4.*mode/(b-2.*mode)

        a3 = (a+1.)**3 * sigma_theta_sqr / b**2
        a2 = (sigma_theta_sqr*(2.*c+1.)*(a+1.)**2) / b**2 - a
        a1 = (a+1.)*(2*c+1.)*c*sigma_theta_sqr/b**2 - c
        a0 = sigma_theta_sqr*(c+1.)*c**2 / b**2

        colorMap = plt.cm.get_cmap('hsv',nMode)
        plt.ylim((-100,100))
        colors = ['r', 'g', 'b']
        for i in range(0, nMode):
                coeffs = [a3[i], a2[i], a1[i], a0[i]]
                roots = np.roots(coeffs)
                print("-- Mode " + str(mode[i]))
                print(" ---> Beta possible values : ")
                print(roots)
                plt.figure(i)
                plt.title("Mode " + str(mode[i]))
                plt.xlabel("Beta")
                plt.ylabel("Values")
                j = 0
                for r in roots:
                        if(not np.iscomplex(r) and r > 0):
                                alpha = ((b + 2.*mode[i])*r - 4*mode[i])/(b - 2.*mode[i])
                                y = scipy.stats.beta.pdf(theta_x, a=alpha, b=r, loc=-0.5*b, scale=b)
                                plt.plot(theta_x,y,lw=2.5,color = colors[j])
                                j+=1
                plt.show()
                #plt.plot(x, a3[i] * x**3 + a2[i] * x**2 + a1[i] * x + a0[i], lw=2.5, color=colorMap(i), label = "Mode " + str(mode[i]))

        plt.legend()
        plt.show()

# ------------- MAIN ---------------------
if __name__ == "__main__":
        plotBetaDefinedByMode()