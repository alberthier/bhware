#!/usr/bin/python
# -*- coding: utf-8 -*-


import sys
from matplotlib.pyplot import *
import math
from math import tan, cos, sin, atan2, pow, sqrt, fabs, pi
import numpy
from scipy.optimize import fsolve, fmin_slsqp



def spline3(t, t1, q, a, b, c, deriv) :
    if (deriv == 2) :
        ret  = ((a - q)/t1) * t + q
    if (deriv == 1) :
        ret  = ((a - q)/(2.0*t1)) * t*t + q * t + b
    if (deriv == 0) :
        ret  = ((a - q)/(6.0*t1)) * t*t*t + (q/2.0) * t*t + b * t + c
        
    return ret
    
def spline34(t ,t1, n, q, a, b, c, deriv) :
    if (deriv == 2) :
        ret  = a*math.pow(t,n+1) - (q * t)/t1 - a*t1*math.pow(t,n) + q
    if (deriv == 1) :
        ret  = (a*math.pow(t,n+2)) / (n+2.0) - (q * pow(t, 2)) / (2.0*t1) - (a*t1*math.pow(t,n+1)) / (n+1.0) + q * t + b
    if (deriv == 0) :
        ret  = (a*math.pow(t,n+3)) / ((n+2.0)*(n+3.0)) - (q * pow(t, 3)) / (6.0*t1) - (a*t1*math.pow(t,n+2)) / ((n+1.0)*(n+2.0)) + (q * pow(t, 2)) / 2.0 + b * t + c
        
    return ret
        
def trajectoire_sp34(t, dcfg_traj, sp_type, deriv) :
    t1 = dcfg_traj['t1']
    if (sp_type == 'sp4_sp4_1') or (sp_type == 'sp4_sp3_1') :
        if (sp_type == 'sp4_sp4_1') :
            n = 1
        if (sp_type == 'sp4_sp3_1') :
            n = 0

        x0 = dcfg_traj['x1_sp4']
        y0 = dcfg_traj['y1_sp4']
        ax = dcfg_traj['ax1_sp4']
        ay = dcfg_traj['ay1_sp4']
        bx = dcfg_traj['bx1_sp4']
        by = dcfg_traj['by1_sp4']
        qx = dcfg_traj['qx1_sp4']
        qy = dcfg_traj['qy1_sp4']
        theta_seg = dcfg_traj['theta_seg']
            
    if (sp_type == 'sp4_sp4_2') or (sp_type == 'sp4_sp3_2'):
        if (sp_type == 'sp4_sp4_2') :
            n = 1
        if (sp_type == 'sp4_sp3_2') :
            n = 0
        y0 = dcfg_traj['y2_sp4']
        x0 = dcfg_traj['x2_sp4']
        ax = dcfg_traj['ax2_sp4']
        ay = dcfg_traj['ay2_sp4']
        bx = dcfg_traj['bx2_sp4']
        by = dcfg_traj['by2_sp4']
        qx = dcfg_traj['qx2_sp4']
        qy = dcfg_traj['qy2_sp4']
        theta_seg = dcfg_traj['theta_seg']
        
    if (sp_type == 'sp4_sp4_n_1') or (sp_type == 'sp4_sp4_n_2') :
        n = 1
        if (sp_type == 'sp4_sp4_n_1') :
            x0 = dcfg_traj['x1_sp4_n']
            y0 = dcfg_traj['y1_sp4_n']
            ax = dcfg_traj['ax1_sp4_n']
            ay = dcfg_traj['ay1_sp4_n']
            bx = dcfg_traj['bx1_sp4_n']
            by = dcfg_traj['by1_sp4_n']
            qx = dcfg_traj['qx1_sp4_n']
            qy = dcfg_traj['qy1_sp4_n']
            theta_seg = dcfg_traj['theta_seg_n']
            
        if (sp_type == 'sp4_sp4_n_2') :
            y0 = dcfg_traj['y2_sp4_n']
            x0 = dcfg_traj['x2_sp4_n']
            ax = dcfg_traj['ax2_sp4_n']
            ay = dcfg_traj['ay2_sp4_n']
            bx = dcfg_traj['bx2_sp4_n']
            by = dcfg_traj['by2_sp4_n']
            qx = dcfg_traj['qx2_sp4_n']
            qy = dcfg_traj['qy2_sp4_n']
            theta_seg = dcfg_traj['theta_seg_n']
            
    x = spline34(t, t1, n, qx, ax, bx, x0, deriv)
    y = spline34(t, t1, n, qy, ay, by, y0, deriv)
    if deriv == 0 :
        [x_r, y_r] = rotation_coord(theta_seg, x-x0, y-y0)
        x_r = x0 + x_r
        y_r = y0 + y_r
    else :
        [x_r, y_r] = rotation_coord(theta_seg, x, y)
    
    return [x_r, y_r]
    
def trajectoire_sp3(t, dcfg_traj, sp_type, deriv) :
    t1 = dcfg_traj['t1']
    if sp_type == 'sp3r_1' :
        x0 = dcfg_traj['x1']
        y0 = dcfg_traj['y1']
        ax = dcfg_traj['ax_sp3r_1']
        ay = dcfg_traj['ay_sp3r_1']
        bx = dcfg_traj['bx_sp3r_1']
        by = dcfg_traj['by_sp3r_1']
    if sp_type == 'sp3r_2' :
        x0 = dcfg_traj['x2']
        y0 = dcfg_traj['y2']
        ax = dcfg_traj['ax_sp3r_2']
        ay = dcfg_traj['ay_sp3r_2']
        bx = dcfg_traj['bx_sp3r_2']
        by = dcfg_traj['by_sp3r_2']
    x = spline3(t, t1, 0, ax, bx, x0, deriv)
    y = spline3(t, t1, 0, ay, by, y0, deriv)
    
    return [x, y]
    
def trajectoire_rotation(t, dcfg_traj, rotation_type) :
    t1 = dcfg_traj['t1']
    if rotation_type == 'r1' :
        x_r = dcfg_traj['x_cr1']
        y_r = dcfg_traj['y_cr1']
        R = 1.0 / dcfg_traj['Rinv_sp3_1']
        theta0 = dcfg_traj['theta0_r1']
        angle = dcfg_traj['angle_r1']
    if rotation_type == 'r2' :
        x_r = dcfg_traj['x_cr2']
        y_r = dcfg_traj['y_cr2']
        R = 1.0 / dcfg_traj['Rinv_sp3_2']
        theta0 = dcfg_traj['theta0_r2']
        angle = dcfg_traj['angle_r2']
        
    theta = theta0 + (t * angle) / t1
    x = x_r + R * cos(theta)
    y = y_r + R * sin(theta)
    
    return [x, y]
    
def rotation_config(dcfg_traj, rotation_type) :
    t1 = dcfg_traj['t1']
    if rotation_type == 'r1' :
        sp = 'sp3r_1'
        ax = dcfg_traj['ax_sp3r_1']
        ay = dcfg_traj['ay_sp3r_1']
        bx = dcfg_traj['bx_sp3r_1']
        by = dcfg_traj['by_sp3r_1']
        x1 = dcfg_traj['x1']
        y1 = dcfg_traj['y1']
    if rotation_type == 'r2' :
        sp = 'sp3r_2'
        ax = dcfg_traj['ax_sp3r_2']
        ay = dcfg_traj['ay_sp3r_2']
        bx = dcfg_traj['bx_sp3r_2']
        by = dcfg_traj['by_sp3r_2']
        x1 = dcfg_traj['x2']
        y1 = dcfg_traj['y2']
        
    pos_f = trajectoire_sp3(t1, dcfg_traj, sp, 0)
    Diff1BS_f = trajectoire_sp3(t1, dcfg_traj, sp, 1)
    if rotation_type == 'r1' :
        sp = 'sp3r_1'
        angle = dcfg_traj['angle_r1']
    if rotation_type == 'r2' :
        sp = 'sp3r_2'
        angle = dcfg_traj['angle_r2']
    Rinv_sp3 = Rinv_courbure(t1, dcfg_traj, sp)
    
    if Rinv_sp3 > 0.0 :
        signe_Rinv_sp3 = 1.0
    else :
        signe_Rinv_sp3 = -1.0
    theta_f = atan2(Diff1BS_f[1], Diff1BS_f[0])
    if (Rinv_sp3 < 0.0) :
        theta_cr = theta_f - pi/2.0
    else :
        theta_cr = theta_f + pi/2.0
        
    x_cr1 = pos_f[0] + cos(theta_cr) / fabs(Rinv_sp3)
    y_cr1 = pos_f[1] + sin(theta_cr) / fabs(Rinv_sp3)
    theta0_r1 = modulo_angle(theta_cr + pi)
    
    theta_fr = modulo_angle(theta_f + angle)
    pos_fr = [x_cr1 + cos(modulo_angle(theta0_r1 + angle)) / fabs(Rinv_sp3)
            , y_cr1 + sin(modulo_angle(theta0_r1 + angle)) / fabs(Rinv_sp3)]
    
    dtheta = signe_Rinv_sp3 * 1e-4*pi
    theta_p1 = modulo_angle(theta0_r1 + angle)
    theta_p1 = modulo_angle(theta_p1 - 2.0*dtheta)
    theta_p2 = modulo_angle(theta_p1 + dtheta)
    theta_p3 = modulo_angle(theta_p2 + dtheta)
    xp1 = x_cr1 + cos(theta_p1) / fabs(Rinv_sp3)
    yp1 = y_cr1 + sin(theta_p1) / fabs(Rinv_sp3)
    xp2 = x_cr1 + cos(theta_p2) / fabs(Rinv_sp3)
    yp2 = y_cr1 + sin(theta_p2) / fabs(Rinv_sp3)
    xp3 = x_cr1 + cos(theta_p3) / fabs(Rinv_sp3)
    yp3 = y_cr1 + sin(theta_p3) / fabs(Rinv_sp3)
    
    dx1 = xp2 - xp1
    dy1 = yp2 - yp1
    dx2 = xp3 - xp2
    dy2 = yp3 - yp2
    #0.82
    qx = 1.0 * (dx2 - dx1) / pow(dtheta / fabs(Rinv_sp3), 2)
    qy = 1.0 * (dy2 - dy1) / pow(dtheta / fabs(Rinv_sp3), 2)
    
    if rotation_type == 'r1' :
        dcfg_traj['x_cr1'] = x_cr1
        dcfg_traj['y_cr1'] = y_cr1
        dcfg_traj['Rinv_sp3_1'] = fabs(Rinv_sp3)
        dcfg_traj['signe_Rinv_sp3_1'] = signe_Rinv_sp3
        dcfg_traj['theta0_r1'] = theta0_r1
        dcfg_traj['qx1_n'] = qx
        dcfg_traj['qy1_n'] = qy
        dcfg_traj['x1_n'] = pos_fr[0]
        dcfg_traj['y1_n'] = pos_fr[1]
        dcfg_traj['theta1_n'] = theta_fr
    if rotation_type == 'r2' :
        dcfg_traj['x_cr2'] = x_cr1
        dcfg_traj['y_cr2'] = y_cr1
        dcfg_traj['Rinv_sp3_2'] = fabs(Rinv_sp3)
        dcfg_traj['signe_Rinv_sp3_2'] = signe_Rinv_sp3
        dcfg_traj['theta0_r2'] = theta0_r1
        dcfg_traj['qx2_n'] = qx
        dcfg_traj['qy2_n'] = qy
        dcfg_traj['x2_n'] = pos_fr[0]
        dcfg_traj['y2_n'] = pos_fr[1]
        dcfg_traj['theta2_n'] = modulo_angle(theta_fr + pi)
    
    return dcfg_traj
   
def Rinv_courbure(t, dcfg_traj, sp_type) :
    if (sp_type == 'sp4_sp4_1') or (sp_type == 'sp4_sp4_2') or (sp_type == 'sp4_sp4_n_1') or (sp_type == 'sp4_sp4_n_2') or (sp_type == 'sp4_sp3_1') or (sp_type == 'sp4_sp3_2'):
        diff1BS = trajectoire_sp34(t, dcfg_traj, sp_type, 1)
        diff2BS = trajectoire_sp34(t, dcfg_traj, sp_type, 2)
        
    if (sp_type == 'sp3r_1') or (sp_type == 'sp3r_2') :
        diff1BS = trajectoire_sp3(t, dcfg_traj, sp_type, 1)
        diff2BS = trajectoire_sp3(t, dcfg_traj, sp_type, 2)

    dl_dt = sqrt(pow(diff1BS[0], 2) + pow(diff1BS[1], 2));

    diffTheta = pow(diff1BS[0], 2) + pow(diff1BS[1], 2);

    # Restriction de la somme precedente a une valeur minimale pour eviter la division par zero et maximiser le resultat final de diffTheta
    if (diffTheta < 1e-9) :
        diffTheta = 1e-9

    diffTheta = ((diff2BS[1] * diff1BS[0]) - (diff1BS[1] * diff2BS[0])) / diffTheta;

    Rinv_courbure = diffTheta / dl_dt;
    
    return Rinv_courbure
    
def vitesse_limite(vmax, ecart_roue, diffThetaTrajectoire) :
    return (vmax / (1.0 + (fabs(diffThetaTrajectoire) * (ecart_roue / 2.0) )));

def signe_delta_xy(theta) :
    if (theta > -pi/2.0) and (theta < pi/2.0) :
        s_dx = 1.0
    else :
        s_dx = -1.0
    
    if (theta > 0.0) :
        s_dy = 1.0
    else :
        s_dy = -1.0
        
    return [s_dx, s_dy]
    
def modulo_angle(theta) :
    theta_mod = theta
    if theta < -math.pi :
        theta_mod = theta + 2*math.pi
    if theta > math.pi :
        theta_mod = theta - 2*math.pi
        
    return theta_mod
    
def initialSplineForCircle(dcfg_traj, sp_type) :
    Da = dcfg_traj['Da']
    Db = dcfg_traj['Db']
    t1 = dcfg_traj['t1']
    if sp_type == 'sp3r_1' :
        x1 = dcfg_traj['x1']
        y1 = dcfg_traj['y1']
        theta1 = dcfg_traj['theta1']
        x2 = dcfg_traj['x2']
        y2 = dcfg_traj['y2']
    if sp_type == 'sp3r_2' :
        x1 = dcfg_traj['x2']
        y1 = dcfg_traj['y2']
        theta1 = modulo_angle(dcfg_traj['theta2'] + pi)
        x2 = dcfg_traj['x1']
        y2 = dcfg_traj['y1']
        
    [ax, ay, bx, by] = s3r_ab(x1, y1, theta1, x2, y2, Db, Da, t1)
    
    if sp_type == 'sp3r_1' :
        dcfg_traj['ax_sp3r_1'] = ax
        dcfg_traj['ay_sp3r_1'] = ay
        dcfg_traj['bx_sp3r_1'] = bx
        dcfg_traj['by_sp3r_1'] = by
    if sp_type == 'sp3r_2' :
        dcfg_traj['ax_sp3r_2'] = ax
        dcfg_traj['ay_sp3r_2'] = ay
        dcfg_traj['bx_sp3r_2'] = bx
        dcfg_traj['by_sp3r_2'] = by
        
    return dcfg_traj
    
def s3r_ab(x1, y1, theta1, x2, y2, Db, Da, t1) :
    theta_seg1 = math.atan2(y2 - y1, x2 - x1)
    
    delta_theta1 = theta1 - theta_seg1
    delta_theta1 = modulo_angle(delta_theta1)
        
    if delta_theta1 > 0.0 :
        theta1_a = theta1 - pi/2.0
    else :
        theta1_a = theta1 + pi/2.0
    theta1_a = modulo_angle(theta1_a)
        
    [s_bx, s_by] = signe_delta_xy(theta1)
    
    [s_ax, s_ay] = signe_delta_xy(theta1_a)
    
    bx = s_bx * ((Db/t1) / sqrt(1.0 + pow(tan(theta1), 2)))
    by = s_by * sqrt(pow(Db/t1, 2) - pow(bx, 2))
    
    ax = s_ax * (((6.0*Da)/pow(t1, 2)) / sqrt(1.0 + pow(tan(theta1_a), 2)))
    ay = s_ay * sqrt(pow((6.0*Da)/pow(t1, 2), 2) - pow(ax, 2))
    
    return ([ax, ay, bx, by])

def rotation_coord(theta, x, y) :
    x_n = x * cos(theta) - y * sin(theta)
    y_n = x * sin(theta) + y * cos(theta)
    return [x_n, y_n]

def sds_ab(dcfg_traj, traj_type) :
    t1 = dcfg_traj['t1']
    
    if traj_type == 'sp4_sp4' :
        n1 = 1
        n2 = 1
        x1 = dcfg_traj['x1']
        y1 = dcfg_traj['y1']
        theta1 = dcfg_traj['theta1']
        x2 = dcfg_traj['x2']
        y2 = dcfg_traj['y2']
        theta2 = dcfg_traj['theta2']
        qx1 = 0.0
        qy1 = 0.0
        qx2 = 0.0
        qy2 = 0.0
        
    if traj_type == 'sp4_sp4_n' :
        n1 = 1
        n2 = 1
        x1 = dcfg_traj['x1_n']
        y1 = dcfg_traj['y1_n']
        theta1 = dcfg_traj['theta1_n']
        x2 = dcfg_traj['x2_n']
        y2 = dcfg_traj['y2_n']
        theta2 = dcfg_traj['theta2_n']
        qx1 = dcfg_traj['qx1_n']
        qy1 = dcfg_traj['qy1_n']
        qx2 = dcfg_traj['qx2_n']
        qy2 = dcfg_traj['qy2_n']
        
    if traj_type == 'sp4_sp3' :
        n1 = 1
        n2 = 0
        x1 = dcfg_traj['x1']
        y1 = dcfg_traj['y1']
        theta1 = dcfg_traj['theta1']
        x2 = dcfg_traj['x2']
        y2 = dcfg_traj['y2']
        theta2 = dcfg_traj['theta2']
        qx1 = 0.0
        qy1 = 0.0
        qx2 = 0.0
        qy2 = 0.0
    
    theta_seg = atan2(y2-y1, x2-x1)
    theta1_base = modulo_angle(theta1 - theta_seg)
    theta2_base = modulo_angle(modulo_angle(theta2 - pi) - theta_seg)
    [x2_base, y2_base] = rotation_coord(-theta_seg, x2, y2)
    [qx1_base, qy1_base] = rotation_coord(-theta_seg, qx1, qy1)
    [qx2_base, qy2_base] = rotation_coord(-theta_seg, qx2, qy2)
    #~ [qx1_base, qy1_base] = [qx1, qy1]
    #~ [qx2_base, qy2_base] = [qx2, qy2]
    
    qx1_base = qx1_base * (fabs(cos(theta1_base))/t1)
    qy1_base = qy1_base * (fabs(sin(theta1_base))/t1)
    qx2_base = qx2_base * (fabs(-cos(theta2_base))/t1)
    qy2_base = qy2_base * (fabs(-sin(theta2_base))/t1)
    
    [ax1, ay1, bx1, by1, ax2, ay2, bx2, by2, qx1, qy1, qx2, qy2] = sds_ab_base(x1, y1, theta1_base, x2_base, theta2_base, t1, n1, n2, qx1_base, qy1_base, qx2_base, qy2_base)
    
    if (traj_type == 'sp4_sp4') or (traj_type == 'sp4_sp3') :
        dcfg_traj['x1_sp4'] = x1
        dcfg_traj['y1_sp4'] = y1
        dcfg_traj['ax1_sp4'] = ax1
        dcfg_traj['ay1_sp4'] = ay1
        dcfg_traj['bx1_sp4'] = bx1
        dcfg_traj['by1_sp4'] = by1
        dcfg_traj['qx1_sp4'] = qx1
        dcfg_traj['qy1_sp4'] = qy1
        
        dcfg_traj['x2_sp4'] = x2
        dcfg_traj['y2_sp4'] = y2
        dcfg_traj['ax2_sp4'] = ax2
        dcfg_traj['ay2_sp4'] = ay2
        dcfg_traj['bx2_sp4'] = bx2
        dcfg_traj['by2_sp4'] = by2
        dcfg_traj['qx2_sp4'] = qx2
        dcfg_traj['qy2_sp4'] = qy2
        
        dcfg_traj['theta_seg'] = theta_seg
        
    if traj_type == 'sp4_sp4_n' :
        dcfg_traj['x1_sp4_n'] = x1
        dcfg_traj['y1_sp4_n'] = y1
        dcfg_traj['ax1_sp4_n'] = ax1
        dcfg_traj['ay1_sp4_n'] = ay1
        dcfg_traj['bx1_sp4_n'] = bx1
        dcfg_traj['by1_sp4_n'] = by1
        dcfg_traj['qx1_sp4_n'] = qx1
        dcfg_traj['qy1_sp4_n'] = qy1
        
        dcfg_traj['x2_sp4_n'] = x2
        dcfg_traj['y2_sp4_n'] = y2
        dcfg_traj['ax2_sp4_n'] = ax2
        dcfg_traj['ay2_sp4_n'] = ay2
        dcfg_traj['bx2_sp4_n'] = bx2
        dcfg_traj['by2_sp4_n'] = by2
        dcfg_traj['qx2_sp4_n'] = qx2
        dcfg_traj['qy2_sp4_n'] = qy2
        
        dcfg_traj['theta_seg_n'] = theta_seg
        
    return (dcfg_traj)

def sds_ab_base(x1, y1, theta1, x2, theta2, t1, n1, n2, qx1_0, qy1_0, qx2_0, qy2_0) :
    
    y2 = y1
    D = x2-x1
    Cmax = 0.1
    if 0.4*D > Cmax :
        C = Cmax
    else :
        C = 0.4*D 
        
    CoefC = 1.0
    qx1 = qx1_0 * C*CoefC
    qy1 = qy1_0 * C*CoefC
    qx2 = qx2_0 * C*CoefC
    qy2 = qy2_0 * C*CoefC
        
    S1 = 1.0
    if theta1 < 0.0 :
        S1 = -S1
    #~ print("S1 : "+str(S1))
        
    S2 = 1.0
    if theta2 < 0.0 :
        S2 = -S2
    #~ print("S2 : "+str(S2))

    
    Tx1 = (t1/2.0) * qx1
    Tx2 = (t1/2.0) * qx2
    Ty1 = (t1/2.0) * qy1
    Ty2 = (t1/2.0) * qy2
    T2x1 = ((t1*t1)/3.0) * qx1
    T2x2 = ((t1*t1)/3.0) * qx2
    T2y1 = ((t1*t1)/3.0) * qy1
    T2y2 = ((t1*t1)/3.0) * qy2    
    A31 = -pow(t1,n1+2) / ((n1+2.0)*(n1+1.0)) 
    A32 = -pow(t1,n2+2) / ((n2+2.0)*(n2+1.0)) 
    A41 = -2.0*pow(t1,n1+3) / ((n1+1.0)*(n1+2.0)*(n1+3.0)) 
    A42 = -2.0*pow(t1,n2+3) / ((n2+1.0)*(n2+2.0)*(n2+3.0)) 
    A01 = T2x1 + S1*T2y1
    A02 = (A42/A32) * ((Tx1+Tx2) - S2*(Ty1+Ty2)) - T2x2 + S2*T2y2
    A11 = t1 * (1.0 + S1*tan(theta1))
    A12 = (A42/A32) * (1.0 - S2 * tan(theta1))
    A13 = ((A42/A32) - t1) * (1.0 - S2 * tan(theta2))
    A14 = (A42*A31) / A32
    A15 = (1.0 - cos(theta1)) / (A31 * cos(theta1))
    A16 = Tx1 / A31
    A17 = A41*A15 + A11
    A18 = A01 - A41*A16
    A19 = A14*A15 + A12
    A20 = A02 - A14*A16
    A21 = 1.0 + A31*A15
    A22 = A31*A16 - (Tx1+Tx2)
    A23 = -((S2*A14*A17)/(S1*A41) + A19) / A13
    A24 = (C - A20 + (S2*A14*(C-A18))/(S1*A41)) / A13
    A25 = -(A21 + A23) / A32
    A26 = (A22 - A24) / A32
    A27 = -A17 / (S1*A41)
    A28 = (C - A18) / (S1*A41)
    A29 = -(A31*A27 + tan(theta1) + tan(theta2)*A23) / A32
    A30 = -(A31*A28 + tan(theta2)*A24 + (Ty1 + Ty2)) / A32
    A50 = 1.0 + A31*A15
    A51 = -A42*A25 - t1*A23 + A41*A15 + t1
    A52 = Tx1
    A53 = -A42*A26 - t1*A24 - A41*A16 + t1 + T2x1 - T2x2 + x1 - x2
    A54 = A31*A27 + tan(theta1)
    A55 = -A42*A29 - t1*tan(theta2)*A23 + A41*A27 + t1*tan(theta1)
    A56 = A31*A28 + Ty1
    A57 = -A42*A30 - t1*tan(theta2)*A24 + A41*A28 - T2y2 - y2 + T2y1 + y1
    A58 = A53*A55 - A51*A57
    A59 = A52*A55 + A53*A54 - A50*A57 - A51*A56
    A60 = A52*A54 - A50*A56
    
    delta_k = pow(A59,2) - 4.0*A58*A60
    k1 = (-A59 + sqrt(delta_k)) / (2.0*A58)
    k2 = (-A59 - sqrt(delta_k)) / (2.0*A58)
    if k1 > 0.0 :
        k = k1
    else :
        k = k2
        
    bx1 = -(A56 + k*A57) / (A54 + k*A55)
    
    ay2 = A29 * bx1 + A30
    
    ax2 = A25 * bx1 + A26
    
    bx2 = A23 * bx1 + A24
    
    ay1 = A27 * bx1 + A28
    
    ax1 = A15 * bx1 - A16
    
    by1 = bx1 * tan(theta1)
    by2 = bx2 * tan(theta2)
        

    #~ print("x_p1(t1)+x_p2(t1) : {0}".format( (A3*ax1 + (t1/2.0)*qx1 + bx1) + (A3*ax2 + (t1/2.0)*qx2 + bx2) ))
    #~ print("y_p1(t1)+y_p2(t1) : {0}".format( (A3*ay1 + (t1/2.0)*qy1 + by1) + (A3*ay2 + (t1/2.0)*qy2 + by2) ))
    
    return ([ax1, ay1, bx1, by1, ax2, ay2, bx2, by2, qx1, qy1, qx2, qy2])

def test_courbure(dcfg_traj) :
    if dcfg_traj['traj_type'] == 'sp4_sp4' :
        t_test = t1/2.0
        Rinv_sp_1 = Rinv_courbure(t_test, dcfg_traj, 'sp4_sp4_1')
        Rinv_sp_2 = Rinv_courbure(t_test, dcfg_traj, 'sp4_sp4_2')
    if dcfg_traj['traj_type'] == 'sp4_sp3' :
        t_test = 0.0
        Rinv_sp_1 = Rinv_courbure(t_test, dcfg_traj, 'sp4_sp3_1')
        Rinv_sp_2 = Rinv_courbure(t_test, dcfg_traj, 'sp4_sp3_2')
    
    v_Rinv_sp_1 = vitesse_limite(vmax, EcartRoue, fabs(Rinv_sp_1))
    v_Rinv_sp_2 = vitesse_limite(vmax, EcartRoue, fabs(Rinv_sp_2))
    
    print("v_Rinv_sp_1 : {0} m/s (ref : {1} m/s)".format(v_Rinv_sp_1, dcfg_traj['v_Rinv_ref']))
    print("v_Rinv_sp_2 : {0} m/s (ref : {1} m/s)".format(v_Rinv_sp_2, dcfg_traj['v_Rinv_ref']))
    
    dcfg_traj['curvature_forced'] = False
    if v_Rinv_sp_1 < v_Rinv_ref :
        dcfg_traj['curvature_forced_1'] = True
        dcfg_traj['angle_r1'] = 0.0
        dcfg_traj['curvature_forced'] = True
    else :
        dcfg_traj['curvature_forced_1'] = False
        dcfg_traj['angle_r1'] = 0.0
        
    if v_Rinv_sp_2 < v_Rinv_ref :
        dcfg_traj['curvature_forced_2'] = True
        dcfg_traj['angle_r2'] = 0.0
        dcfg_traj['curvature_forced'] = True
    else :
        dcfg_traj['curvature_forced_2'] = False
        dcfg_traj['angle_r2'] = 0.0
        
    print("curvature_forced_1 : {0}".format(dcfg_traj['curvature_forced_1']))
    print("curvature_forced_2 : {0}".format(dcfg_traj['curvature_forced_2']))
        
    return dcfg_traj

def generation_curvatureForced(dcfg_traj) :
    # Initialisation
    if dcfg_traj['curvature_forced_1'] == True :
        flag_rotation_angle_value_1 = False
        dcfg_traj = initialSplineForCircle(dcfg_traj, 'sp3r_1')
    else :
        flag_rotation_angle_value_1 = True
    if dcfg_traj['curvature_forced_2'] == True :
        flag_rotation_angle_value_2 = False
        dcfg_traj = initialSplineForCircle(dcfg_traj, 'sp3r_2')
    else :
        flag_rotation_angle_value_2 = True
        
    # Determination des angles de rotation
    while (flag_rotation_angle_value_1 == False) or (flag_rotation_angle_value_2 == False) :
        if flag_rotation_angle_value_1 == False :
        ### rotation 1 ###
            dcfg_traj = rotation_config(dcfg_traj, 'r1')
        
        if flag_rotation_angle_value_2 == False :
            ### rotation 2 ###
            dcfg_traj = rotation_config(dcfg_traj, 'r2')
        
        ### nouvelle s4s4 ###
        dcfg_traj = sds_ab(dcfg_traj, 'sp4_sp4_n')
        
        Rinv_sp4_1_n_0 = Rinv_courbure(0, dcfg_traj, 'sp4_sp4_n_1')
        Rinv_sp4_1_n_1 = Rinv_courbure(0.01*t1, dcfg_traj, 'sp4_sp4_n_1')
        dRinv_sp4_1_n = fabs(Rinv_sp4_1_n_1) - fabs(Rinv_sp4_1_n_0)
        
        Rinv_sp4_2_n_0 = Rinv_courbure(0, dcfg_traj, 'sp4_sp4_n_2')
        Rinv_sp4_2_n_1 = Rinv_courbure(0.01*t1, dcfg_traj, 'sp4_sp4_n_2')
        dRinv_sp4_2_n = fabs(Rinv_sp4_2_n_1) - fabs(Rinv_sp4_2_n_0)
        
        if flag_rotation_angle_value_1 == False :
            if dRinv_sp4_1_n < 0.0 :
                flag_rotation_angle_value_1 = True
            else :
                dcfg_traj['angle_r1'] = dcfg_traj['angle_r1'] + dcfg_traj['signe_Rinv_sp3_1'] * 0.02*pi
            
        if flag_rotation_angle_value_2 == False :
            if dRinv_sp4_2_n < 0.0 :
                flag_rotation_angle_value_2 = True
            else :
                dcfg_traj['angle_r2'] = dcfg_traj['angle_r2'] + dcfg_traj['signe_Rinv_sp3_2'] * 0.02*pi
        
    print("angle_r1 : {0}".format(dcfg_traj['angle_r1']/pi))
    print("angle_r2 : {0}".format(dcfg_traj['angle_r2']/pi))
    
    return dcfg_traj

#########################################################################
#~ x1 = 0.0
#~ y1 = 0.0
#~ theta1 = pi*(0.1)
#~ x2 = 0.3
#~ y2 = 0.7 #0.65*0.8
#~ theta2 = pi*(0.6)

chemin = [[0.0, 0.0, pi*0.1]
        , [0.3, 0.7, pi*0.6] #angle=0.6
        #, [0.0, 1.0, pi*1.0]
        ]
l_dcfg_traj = []
nb_pts = len(chemin)-1

for iSegment in range(nb_pts) :
    pos1 = chemin[iSegment]
    pos2 = chemin[iSegment + 1]
    x1 = pos1[0]
    y1 = pos1[1]
    theta1 = pos1[2]
    x2 = pos2[0]
    y2 = pos2[1]
    theta2 = pos2[2]

    #config
    t1 = 0.1
    vmax = 0.6
    EcartRoue = 0.17
    Rinv_ref = 1.0 / (EcartRoue/2.0)
    v_Rinv_ref = vitesse_limite(vmax, EcartRoue, Rinv_ref)
    Db = 0.03 #0.05
    Da = 0.0018 #0.006
    
    dcfg_traj = {'t1': t1
                , 'v_Rinv_ref': v_Rinv_ref
                , 'Rinv_ref': Rinv_ref
                , 'Db': Db
                , 'Da': Da
                , 'x1': x1
                , 'y1': y1
                , 'theta1': theta1
                , 'x2': x2
                , 'y2': y2
                , 'theta2': theta2
                , 'x1_n': x1
                , 'y1_n': y1
                , 'theta1_n': theta1
                , 'qx1_n': 0.0
                , 'qy1_n': 0.0
                , 'x2_n': x2
                , 'y2_n': y2
                , 'theta2_n': theta2
                , 'qx2_n': 0.0
                , 'qy2_n': 0.0
                , 'traj_type': ''
                , 'curvature_forced': False
                , 'curvature_forced_1': False
                , 'curvature_forced_2': False
                }
    
    if nb_pts == 1 :
        dcfg_traj['traj_type'] = 'sp4_sp4'
        dcfg_traj = sds_ab(dcfg_traj, 'sp4_sp4')
        
        dcfg_traj = test_courbure(dcfg_traj)
        
        if (dcfg_traj['curvature_forced'] == True) :
            dcfg_traj['traj_type'] = 'sp4_sp4_n'
            dcfg_traj = generation_curvatureForced(dcfg_traj)
        
        #~ dcfg_traj['qx1_n'] = 0.0
        #~ dcfg_traj['qy1_n'] = 0.0
        #~ dcfg_traj['qx2_n'] = 0.0
        #~ dcfg_traj['qy2_n'] = 0.0
        #~ dcfg_traj = sds_ab(dcfg_traj, 'sp4_sp4_n')
        
    if nb_pts > 1 :
        if iSegment == 0 :
            dcfg_traj['traj_type'] = 'sp4_sp3'
            dcfg_traj = sds_ab(dcfg_traj, 'sp4_sp3')
            #~ dcfg_traj = test_courbure(dcfg_traj)
            #~ 
            #~ if (dcfg_traj['curvature_forced'] == True) :
                #~ dcfg_traj['traj_type'] = 'sp4_sp4_n'
                #~ dcfg_traj = generation_curvatureForced(dcfg_traj)
    
        
    l_dcfg_traj.append(dcfg_traj)
    #~ sys.exit(2)
    
    
    ### plots ###
    N = 30
    l_t = [(k/(N*1.0))*t1 for k in range(N+1)]
    
    l_x_1 = []
    l_y_1 = []
    l_x_2 = []
    l_y_2 = []
    l_x_sp4_n1 = []
    l_y_sp4_n1 = []
    l_x_sp4_n2 = []
    l_y_sp4_n2 = []
    l_x_sp3_1 = []
    l_y_sp3_1 = []
    l_x_sp3_2 = []
    l_y_sp3_2 = []
    l_x_r1 = []
    l_y_r1 = []
    l_x_r2 = []
    l_y_r2 = []
    l_Rinv_c = []
    l_v_lim = []
    l_v_lim_s2 = []
    l_v_lim_s4_n1 = []
    l_v_lim_s4_n2 = []
    l_v_lim_r = []
    l_v_lim_r2 = []
    
    for t in l_t :
        
        if (dcfg_traj['traj_type'] == 'sp4_sp4') :
            [x_1, y_1] = trajectoire_sp34(t, dcfg_traj, 'sp4_sp4_1',0)
            l_x_1.append(x_1)
            l_y_1.append(y_1)
            
            [x_2, y_2] = trajectoire_sp34(t, dcfg_traj, 'sp4_sp4_2', 0)
            l_x_2.append(x_2)
            l_y_2.append(y_2)
        
        if (dcfg_traj['traj_type'] == 'sp4_sp3') :
            [x_1, y_1] = trajectoire_sp34(t, dcfg_traj, 'sp4_sp3_1',0)
            l_x_1.append(x_1)
            l_y_1.append(y_1)
            
            [x_2, y_2] = trajectoire_sp34(t, dcfg_traj, 'sp4_sp3_2', 0)
            l_x_2.append(x_2)
            l_y_2.append(y_2)
    
        if dcfg_traj['curvature_forced_1'] == True :
            [x_sp3_1, y_sp3_1] = trajectoire_sp3(t, dcfg_traj, 'sp3r_1', 0)
            l_x_sp3_1.append(x_sp3_1)
            l_y_sp3_1.append(y_sp3_1)
            [x_r1, y_r1] = trajectoire_rotation(t, dcfg_traj, 'r1')
            l_x_r1.append(x_r1)
            l_y_r1.append(y_r1)
        
        if dcfg_traj['curvature_forced_2'] == True :
            [x_sp3_2, y_sp3_2] = trajectoire_sp3(t, dcfg_traj, 'sp3r_2', 0)
            l_x_sp3_2.append(x_sp3_2)
            l_y_sp3_2.append(y_sp3_2)
            [x_r2, y_r2] = trajectoire_rotation(t, dcfg_traj, 'r2')
            l_x_r2.append(x_r2)
            l_y_r2.append(y_r2)
        
        if (dcfg_traj['curvature_forced_1'] == True) or (dcfg_traj['curvature_forced_2'] == True) :
            [x_sp4_n1, y_sp4_n1] = trajectoire_sp34(t, dcfg_traj, 'sp4_sp4_n_1',0)
            l_x_sp4_n1.append(x_sp4_n1)
            l_y_sp4_n1.append(y_sp4_n1)
            [x_sp4_n2, y_sp4_n2] = trajectoire_sp34(t, dcfg_traj, 'sp4_sp4_n_2', 0)
            l_x_sp4_n2.append(x_sp4_n2)
            l_y_sp4_n2.append(y_sp4_n2)
        
        if (dcfg_traj['traj_type'] == 'sp4_sp4') or (dcfg_traj['traj_type'] == 'sp4_sp4_n') :
            l_Rinv_c.append(Rinv_courbure(t, dcfg_traj, 'sp4_sp4_1'))
            l_v_lim.append(vitesse_limite(0.6, 0.17, l_Rinv_c[-1]))
            l_v_lim_s2.append(vitesse_limite(0.6, 0.17, Rinv_courbure(t1-t, dcfg_traj, 'sp4_sp4_2')))
        if dcfg_traj['curvature_forced_1'] == True :
            l_v_lim_r.append(vitesse_limite(0.6, 0.17, Rinv_courbure(t, dcfg_traj, 'sp3r_1')))
        if dcfg_traj['curvature_forced_2'] == True :
            l_v_lim_r2.append(vitesse_limite(vmax, 0.17, Rinv_courbure(t1-t, dcfg_traj, 'sp3r_2')))
        if (dcfg_traj['curvature_forced_1'] == True) or (dcfg_traj['curvature_forced_2'] == True) :
            l_v_lim_s4_n1.append(vitesse_limite(vmax, EcartRoue, Rinv_courbure(t, dcfg_traj, 'sp4_sp4_n_1')))
            l_v_lim_s4_n2.append(vitesse_limite(vmax, EcartRoue, Rinv_courbure(t1-t, dcfg_traj, 'sp4_sp4_n_2')))
            
        if (dcfg_traj['traj_type'] == 'sp4_sp3') :
            l_v_lim.append(vitesse_limite(0.6, 0.17, Rinv_courbure(t, dcfg_traj, 'sp4_sp3_1')))
            l_v_lim_s2.append(vitesse_limite(0.6, 0.17, Rinv_courbure(t1-t, dcfg_traj, 'sp4_sp3_2')))
        
    
    figure(1)
    if ((dcfg_traj['traj_type'] == 'sp4_sp4') and (dcfg_traj['curvature_forced'] == False)) or (dcfg_traj['traj_type'] == 'sp4_sp3') :
        plot(l_x_1, l_y_1, 'o')
        plot(l_x_2, l_y_2, 'o')
        plot([l_x_1[-1], l_x_2[-1]], [l_y_1[-1], l_y_2[-1]], '-r')
    
    if dcfg_traj['curvature_forced_1'] == True :
        plot(l_x_sp3_1, l_y_sp3_1, 'o')
        #centre de rotation 1
        plot(dcfg_traj['x_cr1'], dcfg_traj['y_cr1'], 'or')
        plot(l_x_r1, l_y_r1, 'o')
        plot(l_x_r1[0], l_y_r1[0], 'oy')
    
    if dcfg_traj['curvature_forced_2'] == True :
        plot(l_x_sp3_2, l_y_sp3_2, 'o')
        #centre de rotation 2
        plot(dcfg_traj['x_cr2'], dcfg_traj['y_cr2'], 'or')
        plot(l_x_r2, l_y_r2, 'o')
        plot(l_x_r2[0], l_y_r2[0], 'oy')
    
    if (dcfg_traj['curvature_forced'] == True) :
        plot(l_x_sp4_n1, l_y_sp4_n1, 'o')
        plot(l_x_sp4_n2, l_y_sp4_n2, 'o')
        plot([l_x_sp4_n1[-1], l_x_sp4_n2[-1]], [l_y_sp4_n1[-1], l_y_sp4_n2[-1]], '-r')
    
    grid()
    xlim(-0.1, 1.5)
    ylim(-0.1, 1.5)
    
    
    figure(2)
    #~ plot(l_Rinv_c, '-o')
    if (dcfg_traj['traj_type'] == 'sp4_sp4') or (dcfg_traj['traj_type'] == 'sp4_sp4_n') or (dcfg_traj['traj_type'] == 'sp4_sp3') :
        l_v_lim.extend(l_v_lim_s2)
        l_v_lim.extend([0.0])
    if dcfg_traj['curvature_forced_1'] == True :
        l_v_lim.extend(l_v_lim_r)
        l_v_lim.extend([0.0])
    if (dcfg_traj['curvature_forced_1'] == True) or (dcfg_traj['curvature_forced_2'] == True) :
        l_v_lim.extend(l_v_lim_s4_n1)
        l_v_lim.extend([0.0])
        l_v_lim.extend(l_v_lim_s4_n2)
    if dcfg_traj['curvature_forced_2'] == True :
        l_v_lim.extend([0.0])
        l_v_lim.extend(l_v_lim_r2)
    plot(l_v_lim, '-o')
    vmin = vitesse_limite(vmax, EcartRoue, 1.0/(EcartRoue/2.0))
    plot([0, len(l_v_lim)], [vmin, vmin], '-k')
    ylim(0.0, vmax+0.02)
    
    ## temp ##
    grid()
    show()
    ### fin temp ###
    
grid()
show()
    
