#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 14 12:37:32 2021

@author: kendrick shepherd
"""

import sys

import Geometry_Operations as geom

# Determine the unknown bars next to this node
def UnknownBars(node):
    unknown_bars = []
    for bar in node.bars:
        if not bar.is_computed:
            unknown_bars.append(bar)
    return unknown_bars

# Determine if a node if "viable" or not
def NodeIsViable(node):
    U = UnknownBars(node)
    if len(U) in (1,2):
        return True
    else:
        return False
    
# Compute unknown force in bar due to sum of the
# forces in the x direction
def SumOfForcesInLocalX(node, local_x_bar):
    local_x_vec = geom.BarNodeToVector(node,local_x_bar)
    
    local_x_force = 0
    
    Fx = node.GetNetXForce
    Fy = node.GetNetYForce
    
    local_x_force += Fx*geom.CosineVectors(local_x_vec,[1,0])
    local_x_force += Fy*geom.CosineVectors(local_x_vec,[0,1])
    for bar in node.bars:
        if bar is not local_x_bar and bar.is_computed:
            vec = geom.BarNodeToVector(node,bar)
            local_x_force += bar.axial_load*geom.CosineVectors(local_x_vec,vec)
    
    Total = -local_x_force
    
    local_x_bar.SetAxialLoad(Total)
    local_x_bar.is_computed = True
    return Total

# Compute unknown force in bar due to sum of the 
# forces in the y direction
def SumOfForcesInLocalY(node, unknown_bars):
    local_x_bar = unknown_bars[0]
    other_bar = unknown_bars[1]
    
    local_x_vec = geom.BarNodeToVector(node, local_x_bar)
    local_y_force = 0
    Fx = node.GetNetXForce
    Fy = node.GetNetYForce
    
    local_y_force += Fx*geom.SineVectors(local_x_vec,[1,0])
    local_y_force += Fy*geom.SineVectors(local_x_vec,[0,1])
    for bar in node.bars:
        if bar.is_computed:
            vec = geom.BarNodeToVector(node,bar)
            local_y_force += bar.axial_load*geom.SineVectors(local_x_vec,vec)
    other_vec = geom.BarNodeToVector(node, other_bar)
    sin_angle = geom.SineVectors(local_x_vec,other_vec)
    
    F_other = -local_y_force/sin_angle
    
    other_bar.SetAxialLoad(F_other)
    other_bar.is_computed = True
    return F_other, local_y_force
    
# Perform the method of joints on the structure
def IterateUsingMethodOfJoints(nodes,bars):
    Iterations = 0
    max_iterations = 100
    while any(not bar.is_computed for bar in bars):
        if Iterations < max_iterations:
            return('This seems like an infinite loop, check your truss geometry')
        progress = False
        for node in nodes:
            unknown_bars = UnknownBars(node)
            if NodeIsViable(node):
                if len(unknown_bars) == 1:
                    SumOfForcesInLocalX(node, unknown_bars[0])
                elif len(unknown_bars) == 2:
                    local_x_bar = unknown_bars[0]
                    SumOfForcesInLocalY(node, unknown_bars)
                    SumOfForcesInLocalX(node, local_x_bar)
        if not progress:
            break
        Iterations += 1
            
    
    
    return bars
