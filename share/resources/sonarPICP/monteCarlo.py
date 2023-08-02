#!/home/yohan/anaconda3/bin/sudo python

import warnings
import numpy as np
import math
import sys
import matplotlib.pyplot as plt
import scipy.stats as stats
import pptk

"""
    Just a test to see which known distribution could potentially fit the distribution
    obtained for a 3D point computed from its spherical coords with 
     -> r following a normal law
     -> theta (elevation angle) following a beta law
     -> yaw following a normal law

     This file is not used anymore 
"""

def testFitForDistributions(x,data,distributions):
    p_per_dist={}
    for dist_name in distributions:
        print("current dist : " + dist_name)
        try:
            with warnings.catch_warnings():
                warnings.filterwarnings('ignore')
                dist = getattr(stats, dist_name)
                params = dist.fit(data)
                dist_pdf_display = dist.pdf(x,*params)
                #plt.figure(1)
                y , bins,_ = plt.hist(data, bins=100, color='b',density=True)
                #plt.plot(x,dist_pdf_display, color='r')
                #plt.show()

                D, p = stats.kstest(data, cdf=dist_name,args=params)
                print("p-value : " + str(p))

                x_bins = 0.5 * (bins[1:] + bins[:-1])
                dist_pdf = dist.pdf(x_bins, *params)
                sse = np.sum(np.power(dist_pdf - y,2.0))
                print("sse : " + str(sse))

                p_per_dist[dist_name] = [dist_pdf_display, dist_name, p, sse]

        except Exception:
            pass

    return p_per_dist

# Load file with the data
alphas = []
betas  = []
ranges = []
with open("beta_smooth_sparse.txt") as f:
    for line in f:
        line_parsed = line.split(',')
        alphas.append(float(line_parsed[2]))
        betas.append(float(line_parsed[3]))
        ''' range = (range - r_prior) + r_prior '''
        ranges.append(float(line_parsed[4]) + float(line_parsed[5]))

# Dict containing all the distributions 
DISTRIBUTIONS = [
    "alpha", "anglit",
    "arcsine", "beta",
    "betaprime", "bradford",
    "burr", "cauchy",
    "chi", "chi2",
    "cosine", "dgamma",
    "dweibull", "erlang",
    "expon", "exponnorm",
    "exponweib", "exponpow",
    "f", "fatiguelife",
    "fisk", "foldcauchy",
    "foldnorm", "genlogistic",
    "genpareto", "gennorm",
    "genexpon", "genextreme",
     "gamma",
    "gengamma", "genhalflogistic",
    "gilbrat", "gompertz",
    "gumbel_r", "gumbel_l",
    "halfcauchy", "halflogistic",
    "halfnorm", "halfgennorm",
    "hypsecant", "invgamma",
    "invgauss", "invweibull",
    "johnsonsb", "johnsonsu",
    "ksone", "kstwobign",
    "laplace", "levy",
    "levy_l",
    "logistic", "loggamma",
    "loglaplace", "lognorm",
    "lomax", "maxwell",
    "mielke", "nakagami", "norm",
    "pareto," "pearson3",
    "powerlaw," "powerlognorm",
    "powernorm," "rdist",
    "reciprocal", "rayleigh",
    "rice", "recipinvgauss",
    "semicircular","t","truncexpon",
    "truncnorm",
    "uniform", "vonmises",
    "vonmises_line", "wald",
    "weibull_min", "weibull_max",
    "wrapcauchy"
]

# Generate rcos(theta), with r following gamma dist and theta beta distribution 
sonar_resolution = 0.05
var_r = (sonar_resolution/3)**2
print("var_r = " + str(var_r))
var_gamma = 0.000846 # 3sigma = 5 degrees
sigma_gamma = math.sqrt(var_gamma)
n_samples_mc = 10000
b = math.radians(35)
half_b = 0.5*b
K = (1 - math.cos(half_b))/var_r
print("K = " + str(K))
data_size = len(alphas)
count = 0
for alpha,beta,r in zip(alphas,betas,ranges):
    print("threshold value : " + str(K*r))

    k = r**2/var_r
    lamda = var_r/r

    gamma_mean = stats.uniform.rvs(loc=0,scale=2*math.pi)

    print(str(count) + '/' + str(data_size) + " (r = " + str(r) + ", alpha = " + str(alpha) + ", beta = " + str(beta) + ", k = " + str(k) + ", lambda = " + str(lamda) + ")")

    r_samples     = stats.gamma.rvs(a=k,loc=0, scale=lamda,size=n_samples_mc)
    theta_samples = stats.beta.rvs(a=alpha, b=beta, loc= -half_b, scale = b, size=n_samples_mc)
    gamma_samples = stats.norm.rvs(loc=gamma_mean,scale=sigma_gamma,size=n_samples_mc)
    x_samples = []
    y_samples = []
    z_samples = []
    point_samples = []
    for r_s, theta_s, gamma_s in zip(r_samples,theta_samples, gamma_samples):
        c_t = math.cos(theta_s)
        s_t = math.sin(theta_s)
        c_g = math.cos(gamma_s)
        s_g = math.sin(gamma_s)
        x = r_s*c_t*c_g
        y = r_s*c_t*s_g
        z = r_s*s_t
        x_samples.append(x)
        y_samples.append(y)
        z_samples.append(z)
        point_samples.append([x,y,z])

    v = pptk.viewer(point_samples)
    v.set(point_size=0.001, floor_level=0)
    v.set(lookat=[0,0,0])

    plt.subplot(2,3,1)
    plt.xlabel("r")
    plt.ylabel("f_r")
    plt.hist(r_samples, bins=100,density=True)

    plt.subplot(2,3 ,2)
    plt.xlabel("theta")
    plt.ylabel("f_theta")
    plt.hist(theta_samples, bins=100,density=True)

    plt.subplot(2, 3, 3)
    plt.xlabel("gamma")
    plt.ylabel("f_gamma")
    plt.hist(gamma_samples, bins=100,density=True)

    plt.subplot(2, 3, 4)
    plt.xlabel("x")
    plt.ylabel("f_x")
    plt.hist(x_samples, bins=100,density=True)

    plt.subplot(2, 3, 5)
    plt.xlabel("y")
    plt.ylabel("f_y")
    plt.hist(y_samples, bins=100,density=True)

    plt.subplot(2, 3, 6)
    plt.xlabel("z")
    plt.ylabel("f_z")
    z_hist,bins,_ = plt.hist(z_samples, bins=100, color='b',density=True)
    bins_centers = 0.5 * (bins[1:] + bins[:-1])

    plt.figure(3)
    plt.plot(bins_centers,np.log(z_hist))

    '''
    theta_mean = stats.beta.mean(a=alpha, b=beta, loc=-half_b, scale=b)
    theta_var = stats.beta.var(a=alpha, b=beta, loc=-half_b, scale=b)
    print("theta_var : " + str(theta_var))

    z_nfit_mean = r * theta_mean
    z_nfit_var = (r ** 2) * theta_var + (theta_mean ** 2) * var_r + var_r * theta_var
    print("z_nfit = " + str(z_nfit_mean) + "," + str(z_nfit_var))

    z_gammafit_k = z_nfit_mean**2/z_nfit_var
    z_gammafit_lambda = z_nfit_var/z_nfit_mean
    print("z_gammafit : " + str(z_gammafit_k) + "," + str(z_gammafit_lambda))
    '''

    mn, mx = plt.xlim()
    plt.xlim(mn, mx)
    print(mn)
    print(mx)
    x = np.linspace(mn, mx, 1000)

    # Fit z with all distributions 
    p_per_dist = testFitForDistributions(x,z_samples,["norm"]) # DISTRIBUTIONS)
    first = True
    z_best_dist_name = 'uuu'
    colors = ['r', 'g', 'm']
    sorted_p_per_dist = sorted(p_per_dist, key=lambda k: p_per_dist[k][3])
    print(sorted_p_per_dist)

    #z_nfit_params = stats.norm.fit(z_samples)
    #z_gammafit_params = stats.gamma.fit(z_samples)
    #print(z_nfit_params)
    #print(z_gammafit_params)
    #z_nfit = stats.norm.pdf(x,loc=z_nfit_params[0],scale=z_nfit_params[1]) #.pdf(x=x,loc=z_nfit_mean, scale=z_nfit_var)
    #z_gammafit = stats.gamma.pdf(x=x,a=z_gammafit_params[0],loc=z_gammafit_params[1], scale=z_gammafit_params[2])
    #plt.plot(x, z_nfit, color='r')
    #plt.plot(x,z_gammafit, color='g')
    for i in range(0,1):
        plt.plot(x, p_per_dist[sorted_p_per_dist[i]][0], color=colors[i], label=p_per_dist[sorted_p_per_dist[i]][1])
    plt.legend(loc='upper right')

    plt.show()
    wait = input("PRESS ENTER TO CONTINUE.")

    count = count + 1

