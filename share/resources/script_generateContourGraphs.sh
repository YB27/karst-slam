#!/bin/bash

~/anaconda3/bin/python3.7 graph_surface_slices.py dense s matern52 XY exp
~/anaconda3/bin/python3.7 graph_surface_slices.py dense s matern52 XZ exp
~/anaconda3/bin/python3.7 graph_surface_slices.py dense s matern52 ZY exp
~/anaconda3/bin/python3.7 graph_surface_slices.py dense s matern52 XY mat52
~/anaconda3/bin/python3.7 graph_surface_slices.py dense s matern52 XZ mat52
~/anaconda3/bin/python3.7 graph_surface_slices.py dense s matern52 ZY mat52

~/anaconda3/bin/python3.7 graph_surface_slices.py dense yaw matern52Chordal XY free
~/anaconda3/bin/python3.7 graph_surface_slices.py dense yaw matern52Chordal XZ free
~/anaconda3/bin/python3.7 graph_surface_slices.py dense yaw matern52Chordal ZY free
~/anaconda3/bin/python3.7 graph_surface_slices.py dense yaw matern52Chordal XY 50
~/anaconda3/bin/python3.7 graph_surface_slices.py dense yaw matern52Chordal XZ 50
~/anaconda3/bin/python3.7 graph_surface_slices.py dense yaw matern52Chordal ZY 50  
~/anaconda3/bin/python3.7 graph_surface_slices.py sparse s matern52 XY exp
~/anaconda3/bin/python3.7 graph_surface_slices.py sparse s matern52 XZ exp
~/anaconda3/bin/python3.7 graph_surface_slices.py sparse s matern52 ZY exp
~/anaconda3/bin/python3.7 graph_surface_slices.py sparse s matern52 XY mat52
~/anaconda3/bin/python3.7 graph_surface_slices.py sparse s matern52 XZ mat52
~/anaconda3/bin/python3.7 graph_surface_slices.py sparse s matern52 ZY mat52

~/anaconda3/bin/python3.7 graph_surface_slices.py sparse yaw matern52Chordal XY free
~/anaconda3/bin/python3.7 graph_surface_slices.py sparse yaw matern52Chordal XZ free
~/anaconda3/bin/python3.7 graph_surface_slices.py sparse yaw matern52Chordal ZY free
~/anaconda3/bin/python3.7 graph_surface_slices.py sparse yaw matern52Chordal XY 50
~/anaconda3/bin/python3.7 graph_surface_slices.py sparse yaw matern52Chordal XZ 50
~/anaconda3/bin/python3.7 graph_surface_slices.py sparse yaw matern52Chordal ZY 50
