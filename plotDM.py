# -*- coding: utf-8 -*-
#ipython --pylab
import numpy as np
from matplotlib.pyplot import *

file_name = "DynamicModel.csv"

if __name__ == '__main__':
	title = np.loadtxt(file_name, dtype=str, delimiter=",")
	data = np.loadtxt(file_name, skiprows=1, delimiter=",")

	print "title = ", title

	for idx, item in enumerate(data[0,:]):
		plot(data[:,0], data[:,idx], label=title[0, idx])
	legend()
	show()