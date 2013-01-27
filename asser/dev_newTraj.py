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
    
    theta_seg = dcfg_traj['theta_seg']
    
    if (sp_type == 'sp1_type') :
        if (dcfg_traj[sp_type] == 'sp4') or (dcfg_traj[sp_type] == 'sp4_n') :
            n = 1
        if (dcfg_traj[sp_type] == 'sp3') :
            n = 0

        if (dcfg_traj[sp_type] == 'sp4_n') :
            x0 = dcfg_traj['x1_n']
            y0 = dcfg_traj['y1_n']
        else :
            x0 = dcfg_traj['x1']
            y0 = dcfg_traj['y1']
            
        ax = dcfg_traj['ax1']
        ay = dcfg_traj['ay1']
        bx = dcfg_traj['bx1']
        by = dcfg_traj['by1']
        qx = dcfg_traj['qx1']
        qy = dcfg_traj['qy1']
        
    if (sp_type == 'sp2_type') :
        if (dcfg_traj[sp_type] == 'sp4') or (dcfg_traj[sp_type] == 'sp4_n') :
            n = 1
        if (dcfg_traj[sp_type] == 'sp3') :
            n = 0

        if (dcfg_traj[sp_type] == 'sp4_n') :
            x0 = dcfg_traj['x2_n']
            y0 = dcfg_traj['y2_n']
        else :
            x0 = dcfg_traj['x2']
            y0 = dcfg_traj['y2']
            
        ax = dcfg_traj['ax2']
        ay = dcfg_traj['ay2']
        bx = dcfg_traj['bx2']
        by = dcfg_traj['by2']
        qx = dcfg_traj['qx2']
        qy = dcfg_traj['qy2']
        
            
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
    x = spline3(t, t1, 0.0, ax, bx, x0, deriv)
    y = spline3(t, t1, 0.0, ay, by, y0, deriv)
    
    return [x, y]
    
def trajectoire_rotation(t, dcfg_traj, rotation_type, deriv) :
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
    if (deriv == 0) :
        x = x_r + R * cos(theta)
        y = y_r + R * sin(theta)
    if (deriv == 1) :
        x = R * (angle/t1)*(-sin(theta))
        y = R * (angle/t1)*cos(theta)
    if (deriv == 2) :
        x = R * pow(angle/t1, 2)*(-cos(theta))
        y = R * pow(angle/t1, 2)*(-sin(theta))
    
    return [x, y]
    
def init_q(pos_f, theta_f, signe_Rinv_sp3, Rinv_sp3) :
    if (signe_Rinv_sp3*Rinv_sp3 < 0.0) :
        theta_cr = theta_f - pi/2.0
    else :
        theta_cr = theta_f + pi/2.0
        
    x_cr1 = pos_f[0] + cos(theta_cr) / Rinv_sp3
    y_cr1 = pos_f[1] + sin(theta_cr) / Rinv_sp3
    theta0_r1 = modulo_angle(theta_cr + pi)
    
    dtheta = signe_Rinv_sp3 * 1e-4*pi
    #~ theta_p1 = modulo_angle(theta0_r1 + angle)
    theta_p1 = theta0_r1
    theta_p1 = modulo_angle(theta_p1 - 2.0*dtheta)
    theta_p2 = modulo_angle(theta_p1 + dtheta)
    theta_p3 = modulo_angle(theta_p2 + dtheta)
    xp1 = x_cr1 + cos(theta_p1) / Rinv_sp3
    yp1 = y_cr1 + sin(theta_p1) / Rinv_sp3
    xp2 = x_cr1 + cos(theta_p2) / Rinv_sp3
    yp2 = y_cr1 + sin(theta_p2) / Rinv_sp3
    xp3 = x_cr1 + cos(theta_p3) / Rinv_sp3
    yp3 = y_cr1 + sin(theta_p3) / Rinv_sp3
    
    dx1 = xp2 - xp1
    dy1 = yp2 - yp1
    dx2 = xp3 - xp2
    dy2 = yp3 - yp2
    #0.82
    qx = 1.0 * (dx2 - dx1) / pow(dtheta / Rinv_sp3, 2)
    qy = 1.0 * (dy2 - dy1) / pow(dtheta / Rinv_sp3, 2)
    
    return [qx, qy]
    
def rotation_config(dcfg_traj, rotation_type) :
    t1 = dcfg_traj['t1']
    
    if ((dcfg_traj['curve1'] == False) and (rotation_type == 'r1')) or ((dcfg_traj['curve2'] == False) and (rotation_type == 'r2')) :
        if (rotation_type == 'r1') :
            sp = 'sp3r_1'
            ax = dcfg_traj['ax_sp3r_1']
            ay = dcfg_traj['ay_sp3r_1']
            bx = dcfg_traj['bx_sp3r_1']
            by = dcfg_traj['by_sp3r_1']
            x1 = dcfg_traj['x1']
            y1 = dcfg_traj['y1']
            angle = dcfg_traj['angle_r1']
            
        if rotation_type == 'r2' :
            sp = 'sp3r_2'
            ax = dcfg_traj['ax_sp3r_2']
            ay = dcfg_traj['ay_sp3r_2']
            bx = dcfg_traj['bx_sp3r_2']
            by = dcfg_traj['by_sp3r_2']
            x1 = dcfg_traj['x2']
            y1 = dcfg_traj['y2']
            angle = dcfg_traj['angle_r2']
        
        pos_f = trajectoire_sp3(t1, dcfg_traj, sp, 0)
        Diff1BS_f = trajectoire_sp3(t1, dcfg_traj, sp, 1)
        theta_f = atan2(Diff1BS_f[1], Diff1BS_f[0])
            
        Rinv_sp3 = Rinv_courbure(t1, dcfg_traj, sp)
        if Rinv_sp3 > 0.0 :
            signe_Rinv_sp3 = 1.0
        else :
            signe_Rinv_sp3 = -1.0
        Rinv_sp3 = fabs(Rinv_sp3)

    else :
        if rotation_type == 'r1' :
            angle = dcfg_traj['angle_r1']
            pos_f = [dcfg_traj['x1'], dcfg_traj['y1']]
            theta_f = dcfg_traj['theta1']
            signe_Rinv_sp3 = dcfg_traj['s_Rinv_1']
        if rotation_type == 'r2' :
            angle = dcfg_traj['angle_r2']
            pos_f = [dcfg_traj['x2'], dcfg_traj['y2']]
            theta_f = modulo_angle(dcfg_traj['theta2'] + pi)
            signe_Rinv_sp3 = dcfg_traj['s_Rinv_2']
        Rinv_sp3 = dcfg_traj['Rinv_ref']
        
    if (signe_Rinv_sp3*Rinv_sp3 < 0.0) :
        theta_cr = theta_f - pi/2.0
    else :
        theta_cr = theta_f + pi/2.0
        
    x_cr1 = pos_f[0] + cos(theta_cr) / Rinv_sp3
    y_cr1 = pos_f[1] + sin(theta_cr) / Rinv_sp3
    theta0_r1 = modulo_angle(theta_cr + pi)
    
    theta_fr = modulo_angle(theta_f + angle)
    pos_fr = [x_cr1 + cos(modulo_angle(theta0_r1 + angle)) / Rinv_sp3
            , y_cr1 + sin(modulo_angle(theta0_r1 + angle)) / Rinv_sp3]
    
    dtheta = signe_Rinv_sp3 * 1e-4*pi
    theta_p1 = modulo_angle(theta0_r1 + angle)
    theta_p1 = modulo_angle(theta_p1 - 2.0*dtheta)
    theta_p2 = modulo_angle(theta_p1 + dtheta)
    theta_p3 = modulo_angle(theta_p2 + dtheta)
    xp1 = x_cr1 + cos(theta_p1) / Rinv_sp3
    yp1 = y_cr1 + sin(theta_p1) / Rinv_sp3
    xp2 = x_cr1 + cos(theta_p2) / Rinv_sp3
    yp2 = y_cr1 + sin(theta_p2) / Rinv_sp3
    xp3 = x_cr1 + cos(theta_p3) / Rinv_sp3
    yp3 = y_cr1 + sin(theta_p3) / Rinv_sp3
    
    dx1 = xp2 - xp1
    dy1 = yp2 - yp1
    dx2 = xp3 - xp2
    dy2 = yp3 - yp2
    #0.82
    qx = 1.0 * (dx2 - dx1) / pow(dtheta / Rinv_sp3, 2)
    qy = 1.0 * (dy2 - dy1) / pow(dtheta / Rinv_sp3, 2)
    
    if rotation_type == 'r1' :
        dcfg_traj['x_cr1'] = x_cr1
        dcfg_traj['y_cr1'] = y_cr1
        dcfg_traj['Rinv_sp3_1'] = Rinv_sp3
        dcfg_traj['signe_Rinv_sp3_1'] = signe_Rinv_sp3
        dcfg_traj['theta0_r1'] = theta0_r1
        dcfg_traj['qx1'] = qx
        dcfg_traj['qy1'] = qy
        dcfg_traj['x1_n'] = pos_fr[0]
        dcfg_traj['y1_n'] = pos_fr[1]
        dcfg_traj['theta1_n'] = theta_fr
    if rotation_type == 'r2' :
        dcfg_traj['x_cr2'] = x_cr1
        dcfg_traj['y_cr2'] = y_cr1
        dcfg_traj['Rinv_sp3_2'] = Rinv_sp3
        dcfg_traj['signe_Rinv_sp3_2'] = signe_Rinv_sp3
        dcfg_traj['theta0_r2'] = theta0_r1
        dcfg_traj['qx2'] = qx
        dcfg_traj['qy2'] = qy
        dcfg_traj['x2_n'] = pos_fr[0]
        dcfg_traj['y2_n'] = pos_fr[1]
        dcfg_traj['theta2_n'] = modulo_angle(theta_fr + pi)
        if (dcfg_traj['curve2'] == True) :
            dcfg_traj['Rinv_final'] = Rinv_sp3
            dcfg_traj['s_Rinv_final'] = -signe_Rinv_sp3
    
    return dcfg_traj
   
def Rinv_courbure(t, dcfg_traj, sp_type) :    
    if (sp_type == 'sp1_type') or (sp_type == 'sp2_type') :
        if (dcfg_traj[sp_type] == 'sp4') or (dcfg_traj[sp_type] == 'sp3') or (dcfg_traj[sp_type] == 'sp4_n'):
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
    print("s_bx: {0}, s_by: {1}".format(s_bx, s_by))
    
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

def sds_ab(dcfg_traj) :
    t1 = dcfg_traj['t1']
    
    if (dcfg_traj['sp1_type'] == 'sp4') or (dcfg_traj['sp1_type'] == 'sp3'):
        if (dcfg_traj['sp1_type'] == 'sp4') :
            n1 = 1
        if (dcfg_traj['sp1_type'] == 'sp3') :
            n1 = 0
        x1 = dcfg_traj['x1']
        y1 = dcfg_traj['y1']
        theta1 = dcfg_traj['theta1']
        qx1 = 0.0
        qy1 = 0.0
        
    if (dcfg_traj['sp2_type'] == 'sp4') or (dcfg_traj['sp2_type'] == 'sp3'):
        if (dcfg_traj['sp2_type'] == 'sp4') :
            n2 = 1
        if (dcfg_traj['sp2_type'] == 'sp3') :
            n2 = 0
        n2 = 1
        x2 = dcfg_traj['x2']
        y2 = dcfg_traj['y2']
        theta2 = dcfg_traj['theta2']
        qx2 = 0.0
        qy2 = 0.0
        
    if (dcfg_traj['sp1_type'] == 'sp4_n') :
        n1 = 1
        x1 = dcfg_traj['x1_n']
        y1 = dcfg_traj['y1_n']
        theta1 = dcfg_traj['theta1_n']
        qx1 = dcfg_traj['qx1']
        qy1 = dcfg_traj['qy1']
        
    if (dcfg_traj['sp2_type'] == 'sp4_n') :
        n2 = 1
        x2 = dcfg_traj['x2_n']
        y2 = dcfg_traj['y2_n']
        theta2 = dcfg_traj['theta2_n']
        qx2 = dcfg_traj['qx2']
        qy2 = dcfg_traj['qy2']
        
    
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
    
    bx1 = dcfg_traj['Rb_prec'] * cos(theta1_base)
    print("Rb_prec: {0}, bx1: {1}".format(dcfg_traj['Rb_prec'], bx1))
    
    [ax1, ay1, bx1, by1, ax2, ay2, bx2, by2, qx1, qy1, qx2, qy2] = sds_ab_base(x1, y1, theta1_base, x2_base, theta2_base, t1, n1, n2, qx1_base, qy1_base, qx2_base, qy2_base, bx1)
    
    dcfg_traj['ax1'] = ax1
    dcfg_traj['ay1'] = ay1
    dcfg_traj['bx1'] = bx1
    dcfg_traj['by1'] = by1
    dcfg_traj['qx1'] = qx1
    dcfg_traj['qy1'] = qy1
    
    dcfg_traj['ax2'] = ax2
    dcfg_traj['ay2'] = ay2
    dcfg_traj['bx2'] = bx2
    dcfg_traj['by2'] = by2
    dcfg_traj['qx2'] = qx2
    dcfg_traj['qy2'] = qy2
    
    dcfg_traj['theta_seg'] = theta_seg
    
    if theta1 - theta_seg > 0.0 :
        dcfg_traj['s_Rinv_1'] = -1.0
    else :
        dcfg_traj['s_Rinv_1'] = 1.0
        
    if theta2 - theta_seg > 0.0 :
        dcfg_traj['s_Rinv_2'] = -1.0
    else :
        dcfg_traj['s_Rinv_2'] = 1.0
        
    Rinv_final = Rinv_courbure(0.0, dcfg_traj, 'sp2_type')
    if Rinv_final > 0.0 :
        signe_Rinv_final = 1.0
    else :
        signe_Rinv_final = -1.0
    Rinv_final = fabs(Rinv_final)
    dcfg_traj['Rinv_final'] = Rinv_final
    dcfg_traj['s_Rinv_final'] = signe_Rinv_final
        
    return (dcfg_traj)

def sds_ab_base(x1, y1, theta1, x2, theta2, t1, n1, n2, qx1_0, qy1_0, qx2_0, qy2_0, bx1) :
    
    y2 = y1
    D = fabs(x2-x1)
    Cmax = 0.04
    Cmin = 0.01
    if 0.05*D > Cmax :
        C = Cmax
    else :
        C = 0.05*D 
        if C < Cmin :
            C = Cmin
    print(C)
    #~ C=0.04
        
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
    
    if bx1 > 0.0 :
        # determination de k a partir de bx1(=x_prime(t=0)=-x_prime_precedent(t=0))
        k = -(A56 + A54 * bx1) / (A57 + A55 * bx1)
    else :
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
    if (dcfg_traj['sp1_type'] == 'sp4') or (dcfg_traj['sp1_type'] == 'sp4_n'):
        t_test = t1/2.0
    if dcfg_traj['sp1_type'] == 'sp3' :
        t_test = 0.0

    Rinv_sp_1 = Rinv_courbure(t_test, dcfg_traj, 'sp1_type')
    
    if (dcfg_traj['sp2_type'] == 'sp4') or (dcfg_traj['sp2_type'] == 'sp4_n'):
        t_test = t1/2.0
    if dcfg_traj['sp2_type'] == 'sp3' :
        t_test = 0.0

    Rinv_sp_2 = Rinv_courbure(t_test, dcfg_traj, 'sp2_type')
    
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
        print("curve1: {0}".format(dcfg_traj['curve1']))
        if dcfg_traj['curve1'] == False :
            print("### initialSplineForCircle 1")
            dcfg_traj = initialSplineForCircle(dcfg_traj, 'sp3r_1')
        dcfg_traj['sp1_type'] = 'sp4_n'
    else :
        flag_rotation_angle_value_1 = True
        
    if dcfg_traj['curvature_forced_2'] == True :
        flag_rotation_angle_value_2 = False
        if dcfg_traj['curve2'] == False :
            print("### initialSplineForCircle 2")
            dcfg_traj = initialSplineForCircle(dcfg_traj, 'sp3r_2')
        dcfg_traj['sp2_type'] = 'sp4_n'
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
        dcfg_traj = sds_ab(dcfg_traj)
        
        if flag_rotation_angle_value_1 == False :
            Rinv_sp4_1_n_0 = Rinv_courbure(0, dcfg_traj, 'sp1_type')
            Rinv_sp4_1_n_1 = Rinv_courbure(0.01*t1, dcfg_traj, 'sp1_type')
            dRinv_sp4_1_n = fabs(Rinv_sp4_1_n_1) - fabs(Rinv_sp4_1_n_0)
        
        if flag_rotation_angle_value_2 == False :
            Rinv_sp4_2_n_0 = Rinv_courbure(0, dcfg_traj, 'sp2_type')
            Rinv_sp4_2_n_1 = Rinv_courbure(0.01*t1, dcfg_traj, 'sp2_type')
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

#~ chemin = [[0.0, 0.0, pi*0.1]
        #~ , [0.3, 0.7, pi*0.6] #angle=0.6
        #~ , [0.0, 1.0, pi*1.0]
        #~ , [-0.2, 1.2, pi*0.5]
        #~ ]
E = 0.05      
chemin = [[0.2, 1.0, 0.0]
        , [0.9, 1.05-E, 0.0] #angle=0.6
        , [1.05, 0.8+E, 0.0]
        , [1.2, 1.05-E, 0.0]
        , [1.35, 0.8+E, 0.0]
        ]
        
l_dcfg_traj = []
nb_pts = len(chemin)-1
l_v_lim = []
l_xp = []
l_yp = []
l_xs = []
l_ys = []
l_dl_dt = []
l_d2l_dt2 = []

for iSegment in range(nb_pts) :
    print("##### segment #####")
    pos1 = chemin[iSegment]
    pos2 = chemin[iSegment + 1]
    x1 = pos1[0]
    y1 = pos1[1]
    theta1 = pos1[2]
    x2 = pos2[0]
    y2 = pos2[1]
    theta2 = pos2[2]
    theta_seg = atan2(y2 - y1, x2 - x1)

    #config
    t1 = 0.1
    vmax = 0.6
    EcartRoue = 0.17
    Rinv_ref = 1.0 / (EcartRoue/2.0)
    v_Rinv_ref = vitesse_limite(vmax, EcartRoue, Rinv_ref)
    Db = 0.017 #0.03 #0.05
    Da = 0.0006 #0.0018 #0.006
    
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
                , 'qx1': 0.0
                , 'qy1': 0.0
                , 'x2_n': x2
                , 'y2_n': y2
                , 'theta2_n': theta2
                , 'qx2': 0.0
                , 'qy2': 0.0
                , 'Rb_prec': -1.0
                , 'sp1_type': ''
                , 'sp2_type': ''
                , 'curve1': False
                , 'curve2': False
                , 'curvature_forced': False
                , 'curvature_forced_1': False
                , 'curvature_forced_2': False
                , 'curvature_forced_2_prec': False
                }
                
    dcfg_traj['inflexion_point'] = False
    if iSegment < (nb_pts-1) :
        pos2_next = chemin[iSegment + 2]
        x2_next = pos2_next[0]
        y2_next = pos2_next[1]
        theta_seg_next = atan2(y2_next - y2, x2_next - x2)
        if (theta2 - theta_seg) * (theta_seg_next - theta2) < 0.0 :
            dcfg_traj['inflexion_point'] = True
    print("pt d'inflection: {0}".format(dcfg_traj['inflexion_point']))
    
    if nb_pts == 1 :
        dcfg_traj['curve1'] = False
        dcfg_traj['curve2'] = False
        dcfg_traj['sp1_type'] = 'sp4'
        dcfg_traj['sp2_type'] = 'sp4'
        
    if nb_pts > 1 :
        if iSegment == 0 :
            dcfg_traj['curve1'] = False
            dcfg_traj['curve2'] = True
            dcfg_traj['sp1_type'] = 'sp4'
            dcfg_traj['sp2_type'] = 'sp3'
        else :
            # Initialiser la courbure induite par la fin du segment precedent
            dcfg_traj_p = l_dcfg_traj[-1]
            [dcfg_traj['qx1'], dcfg_traj['qy1']] = init_q([dcfg_traj['x1'], dcfg_traj['y1']], dcfg_traj['theta1'], dcfg_traj_p['s_Rinv_final'], dcfg_traj_p['Rinv_final'])
            if dcfg_traj_p['curve2'] == False :
                dcfg_traj['qx1'] = 0.0
                dcfg_traj['qy1'] = 0.0            
                
            if dcfg_traj_p['curvature_forced_2'] == False :
                angle_temp = modulo_angle(modulo_angle(dcfg_traj_p['theta2'] - pi) - dcfg_traj_p['theta_seg'])
                dcfg_traj['Rb_prec'] = dcfg_traj_p['bx2'] / cos(angle_temp)
            else :
                dcfg_traj_p['curvature_forced_2_prec'] = True
            
            # Dernier segment
            if iSegment == (nb_pts - 1) :
                dcfg_traj['curve1'] = True
                if dcfg_traj_p['curve2'] == False :
                    dcfg_traj['curve1'] = False
                dcfg_traj['curve2'] = False
                dcfg_traj['sp1_type'] = 'sp4_n'
                dcfg_traj['sp2_type'] = 'sp4'
            else :
                dcfg_traj['curve1'] = True
                print("curve2_p: {0}".format(dcfg_traj_p['curve2']))
                if dcfg_traj_p['curve2'] == False :
                    dcfg_traj['curve1'] = False
                dcfg_traj['curve2'] = True
                dcfg_traj['sp1_type'] = 'sp4_n'
                dcfg_traj['sp2_type'] = 'sp3'
                
        if dcfg_traj['inflexion_point'] == True :
            dcfg_traj['curve2'] = False
            dcfg_traj['sp2_type'] = 'sp4'
    
    dcfg_traj = sds_ab(dcfg_traj)
    dcfg_traj = test_courbure(dcfg_traj)
    if (dcfg_traj['curvature_forced'] == True) :
        print("## generation_curvatureForced")
        dcfg_traj = generation_curvatureForced(dcfg_traj)
        
    if dcfg_traj['curvature_forced_2'] == False :
        dcfg_traj['curve2'] = False
        print("curve2 affect: {0}".format(dcfg_traj['curve2']))
        
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
    #~ l_v_lim = []
    l_v_lim_s1 = []
    l_v_lim_s2 = []
    l_v_lim_s4_n1 = []
    l_v_lim_s4_n2 = []
    l_v_lim_r = []
    l_v_lim_r2 = []
    
    l_xp_spr_1 = []
    l_xp_r1 = []
    l_xp_s1 = []
    l_xp_s2 = []
    l_xp_r2 = []
    l_xp_spr_2 = []
    l_yp_spr_1 = []
    l_yp_r1 = []
    l_yp_s1 = []
    l_yp_s2 = []
    l_yp_r2 = []
    l_yp_spr_2 = []
    
    l_xs_spr_1 = []
    l_xs_r1 = []
    l_xs_s1 = []
    l_xs_s2 = []
    l_xs_r2 = []
    l_xs_spr_2 = []
    l_ys_spr_1 = []
    l_ys_r1 = []
    l_ys_s1 = []
    l_ys_s2 = []
    l_ys_r2 = []
    l_ys_spr_2 = []
    
    d_l_dl_dt = {'spr_1': [], 'r1': [], 's1': [], 's2': [], 'r2': [], 'spr_2': []}
    d_l_d2l_dt2 = {'spr_1': [], 'r1': [], 's1': [], 's2': [], 'r2': [], 'spr_2': []}
    
    
    for t in l_t :
        ######
        ### Positions sur la trajectoire ###
        ###
        cnonzero = 1e-12
        if dcfg_traj['curvature_forced_1'] == True :
            if dcfg_traj['curve1'] == False :
                [x_sp3_1, y_sp3_1] = trajectoire_sp3(t, dcfg_traj, 'sp3r_1', 0)
                l_x_sp3_1.append(x_sp3_1)
                l_y_sp3_1.append(y_sp3_1)
                [xp, yp] = trajectoire_sp3(t, dcfg_traj, 'sp3r_1', 1)
                dl_dt = sqrt(pow(xp, 2) + pow(yp, 2)) + cnonzero
                d_l_dl_dt['spr_1'].append(dl_dt)
                l_xp_spr_1.append(xp/dl_dt)
                l_yp_spr_1.append(yp/dl_dt)
                [xs, ys] = trajectoire_sp3(t, dcfg_traj, 'sp3r_1', 2)
                d2l_dt2 = sqrt(pow(xs, 2) + pow(ys, 2)) + cnonzero
                d_l_d2l_dt2['spr_1'].append(d2l_dt2)
                l_xs_spr_1.append(xs/d2l_dt2)
                l_ys_spr_1.append(ys/d2l_dt2)
            if (fabs(dcfg_traj['angle_r1']) > 1e-6) :
                [x_r1, y_r1] = trajectoire_rotation(t, dcfg_traj, 'r1', 0)
                l_x_r1.append(x_r1)
                l_y_r1.append(y_r1)
                [xp, yp] = trajectoire_rotation(t, dcfg_traj, 'r1', 1)
                dl_dt = sqrt(pow(xp, 2) + pow(yp, 2)) + cnonzero
                d_l_dl_dt['r1'].append(dl_dt)
                l_xp_r1.append(xp/dl_dt)
                l_yp_r1.append(yp/dl_dt)
                [xs, ys] = trajectoire_rotation(t, dcfg_traj, 'r1', 2)
                d2l_dt2 = sqrt(pow(xs, 2) + pow(ys, 2)) + cnonzero
                d_l_d2l_dt2['r1'].append(d2l_dt2)
                l_xs_r1.append(xp/d2l_dt2)
                l_ys_r1.append(yp/d2l_dt2)
        
        [x_1, y_1] = trajectoire_sp34(t, dcfg_traj, 'sp1_type',0)
        l_x_1.append(x_1)
        l_y_1.append(y_1)
        [xp, yp] = trajectoire_sp34(t, dcfg_traj, 'sp1_type',1)
        dl_dt = sqrt(pow(xp, 2) + pow(yp, 2)) + cnonzero
        d_l_dl_dt['s1'].append(dl_dt)
        l_xp_s1.append(xp/dl_dt)
        l_yp_s1.append(yp/dl_dt)
        [xs, ys] = trajectoire_sp34(t, dcfg_traj, 'sp1_type',2)
        d2l_dt2 = sqrt(pow(xs, 2) + pow(ys, 2)) + cnonzero
        d_l_d2l_dt2['s1'].append(d2l_dt2)
        l_xs_s1.append(xs/d2l_dt2)
        l_ys_s1.append(ys/d2l_dt2)
        
        [x_2, y_2] = trajectoire_sp34(t1-t, dcfg_traj, 'sp2_type', 0)
        l_x_2.append(x_2)
        l_y_2.append(y_2)
        [xp, yp] = trajectoire_sp34(t1-t, dcfg_traj, 'sp2_type', 1)
        dl_dt = sqrt(pow(xp, 2) + pow(yp, 2)) + cnonzero
        d_l_dl_dt['s2'].append(dl_dt)
        l_xp_s2.append(-xp/dl_dt)
        l_yp_s2.append(-yp/dl_dt)
        [xs, ys] = trajectoire_sp34(t1-t, dcfg_traj, 'sp2_type', 2)
        d2l_dt2 = sqrt(pow(xs, 2) + pow(ys, 2)) + cnonzero
        d_l_d2l_dt2['s2'].append(d2l_dt2)
        l_xs_s2.append(xs/d2l_dt2)
        l_ys_s2.append(ys/d2l_dt2)
        
        if dcfg_traj['curvature_forced_2'] == True :
            if (fabs(dcfg_traj['angle_r2']) > 1e-6) :
                [x_r2, y_r2] = trajectoire_rotation(t1-t, dcfg_traj, 'r2', 0)
                l_x_r2.append(x_r2)
                l_y_r2.append(y_r2)
                [xp, yp] = trajectoire_rotation(t1-t, dcfg_traj, 'r2', 1)
                dl_dt = sqrt(pow(xp, 2) + pow(yp, 2)) + cnonzero
                d_l_dl_dt['r2'].append(dl_dt)
                l_xp_r2.append(-xp/dl_dt)
                l_yp_r2.append(-yp/dl_dt)
                [xs, ys] = trajectoire_rotation(t1-t, dcfg_traj, 'r2', 2)
                d2l_dt2 = sqrt(pow(xs, 2) + pow(ys, 2)) + cnonzero
                d_l_d2l_dt2['r2'].append(d2l_dt2)
                l_xs_r2.append(xs/d2l_dt2)
                l_ys_r2.append(ys/d2l_dt2)
            if dcfg_traj['curve2'] == False :
                [x_sp3_2, y_sp3_2] = trajectoire_sp3(t1-t, dcfg_traj, 'sp3r_2', 0)
                l_x_sp3_2.append(x_sp3_2)
                l_y_sp3_2.append(y_sp3_2)
                [xp, yp] = trajectoire_sp3(t1-t, dcfg_traj, 'sp3r_2', 1)
                dl_dt = sqrt(pow(xp, 2) + pow(yp, 2)) + cnonzero
                d_l_dl_dt['spr_2'].append(dl_dt)
                l_xp_spr_2.append(-xp/dl_dt)
                l_yp_spr_2.append(-yp/dl_dt)
                [xs, ys] = trajectoire_sp3(t1-t, dcfg_traj, 'sp3r_2', 2)
                d2l_dt2 = sqrt(pow(xs, 2) + pow(ys, 2)) + cnonzero
                d_l_d2l_dt2['spr_2'].append(d2l_dt2)
                l_xs_spr_2.append(xs/d2l_dt2)
                l_ys_spr_2.append(ys/d2l_dt2)
        
        ######
        ### Vitesse limite du a la courbure tout le long de la trajectoire ###
        ###
        l_Rinv_c.append(Rinv_courbure(t, dcfg_traj, 'sp1_type'))
        l_v_lim_s1.append(vitesse_limite(0.6, 0.17, l_Rinv_c[-1]))
        l_v_lim_s2.append(vitesse_limite(0.6, 0.17, Rinv_courbure(t1-t, dcfg_traj, 'sp2_type')))
        
        if dcfg_traj['curvature_forced_1'] == True :
            if dcfg_traj['curve1'] == False :
                l_v_lim_r.append(vitesse_limite(0.6, 0.17, Rinv_courbure(t, dcfg_traj, 'sp3r_1')))
        if dcfg_traj['curvature_forced_2'] == True :
            if dcfg_traj['curve2'] == False :
                l_v_lim_r2.append(vitesse_limite(vmax, 0.17, Rinv_courbure(t1-t, dcfg_traj, 'sp3r_2')))
                
    ######
    ### Plot de la trajectoire ###
    ###
    figure(1)
    if dcfg_traj['curvature_forced_1'] == True :
        if dcfg_traj['curve1'] == False :
            plot(l_x_sp3_1, l_y_sp3_1, 'ob')
            if len(l_xp) > 0 :
                l_xp.extend(l_xp_spr_1[1:])
                l_yp.extend(l_yp_spr_1[1:])
                l_xs.extend(l_xs_spr_1[1:])
                l_ys.extend(l_ys_spr_1[1:])
            else :
                l_xp.extend(l_xp_spr_1)
                l_yp.extend(l_yp_spr_1)
                l_xs.extend(l_xs_spr_1)
                l_ys.extend(l_ys_spr_1)
            l_dl_dt.extend(d_l_dl_dt['spr_1'])
            l_d2l_dt2.extend(d_l_dl_dt['spr_1'])
        if (fabs(dcfg_traj['angle_r1']) > 1e-6) :
            #centre de rotation 1
            plot(dcfg_traj['x_cr1'], dcfg_traj['y_cr1'], 'or')
            plot(l_x_r1, l_y_r1, 'oy')
            plot(l_x_r1[0], l_y_r1[0], 'oy')
            l_xp.extend(l_xp_r1[1:])
            l_yp.extend(l_yp_r1[1:])
            l_xs.extend(l_xs_r1[1:])
            l_ys.extend(l_ys_r1[1:])
            l_dl_dt.extend(d_l_dl_dt['r1'])
            l_d2l_dt2.extend(d_l_dl_dt['r1'])
    
    plot(l_x_1, l_y_1, 'or')
    if len(l_xp) > 0 :
        l_xp.extend(l_xp_s1[1:])
        l_yp.extend(l_yp_s1[1:])
        l_xs.extend(l_xs_s1[1:])
        l_ys.extend(l_ys_s1[1:])
    else :
        l_xp.extend(l_xp_s1)
        l_yp.extend(l_yp_s1)
        l_xs.extend(l_xs_s1)
        l_ys.extend(l_ys_s1)
    l_dl_dt.extend(d_l_dl_dt['s1'])
    l_d2l_dt2.extend(d_l_dl_dt['s1'])
    plot(l_x_2, l_y_2, 'or')
    l_xp.extend(l_xp_s2[1:])
    l_yp.extend(l_yp_s2[1:])
    l_xs.extend(l_xs_s2[1:])
    l_ys.extend(l_ys_s2[1:])
    l_dl_dt.extend(d_l_dl_dt['s2'])
    l_d2l_dt2.extend(d_l_dl_dt['s2'])
    plot([l_x_1[-1], l_x_2[0]], [l_y_1[-1], l_y_2[0]], '-r')
    
    if dcfg_traj['curvature_forced_2'] == True :
        if (fabs(dcfg_traj['angle_r2']) > 1e-6) :
            #centre de rotation 2
            plot(dcfg_traj['x_cr2'], dcfg_traj['y_cr2'], 'or')
            plot(l_x_r2, l_y_r2, 'oy')
            l_xp.extend(l_xp_r2[1:])
            l_yp.extend(l_yp_r2[1:])
            l_xs.extend(l_xs_r2[1:])
            l_ys.extend(l_ys_r2[1:])
            l_dl_dt.extend(d_l_dl_dt['r2'])
            l_d2l_dt2.extend(d_l_dl_dt['r2'])
            plot(l_x_r2[0], l_y_r2[0], 'oy')
        if dcfg_traj['curve2'] == False :
            plot(l_x_sp3_2, l_y_sp3_2, 'ob')
            l_xp.extend(l_xp_spr_2[1:])
            l_yp.extend(l_yp_spr_2[1:])
            l_xs.extend(l_xs_spr_2[1:])
            l_ys.extend(l_ys_spr_2[1:])
            l_dl_dt.extend(d_l_dl_dt['spr_2'])
            l_d2l_dt2.extend(d_l_dl_dt['spr_2'])
    
    grid()
    xlim(-0.3, 1.5)
    ylim(-0.1, 1.5)
    
    ######
    ### Enregistrement des vitesses limites ###
    ###
    if dcfg_traj['curvature_forced_1'] == True :
        if dcfg_traj['curve1'] == False :
            l_v_lim.extend(l_v_lim_r)
            #~ l_v_lim.extend([0.0])
    
    l_v_lim.extend(l_v_lim_s1)
    l_v_lim.extend(l_v_lim_s2)
    #~ l_v_lim.extend([0.0])
        
    if dcfg_traj['curvature_forced_2'] == True :
        if dcfg_traj['curve2'] == False :
            #~ l_v_lim.extend([0.0])
            l_v_lim.extend(l_v_lim_r2)
    l_v_lim.extend([0.0])
    
title('Trajectoire')
xlabel('x (m)')
ylabel('y (m)')
grid()
        
figure(2)
plot(l_v_lim, '-o')
vmin = vitesse_limite(vmax, EcartRoue, 1.0/(EcartRoue/2.0))
plot([0, len(l_v_lim)], [vmin, vmin], '-k')
title('Vitesse max admissible')
ylabel('v_lim (m/s)')
ylim(0.0, vmax+0.02)
grid()

## temp ##
#~ grid()
#~ show()
### fin temp ###
    

figure(3)
subplot(211)
plot(l_xp, '-o')
ylabel('dx/dl')
title('Derives premieres')
grid()
subplot(212)
plot(l_yp, '-o')
ylabel('dy/dl')
grid()

figure(4)
subplot(211)
plot(l_xs, '-o')
ylabel('d2x/dl2')
title('Derives secondes')
grid()
subplot(212)
plot(l_ys, '-o')
ylabel('d2y/dl2')
grid()

figure(5)
#~ subplot(211)
plot(l_dl_dt, '-o')
ylabel('dl/dt')
title('Facteur de correction')
grid()
#~ subplot(212)
#~ plot(l_d2l_dt2, '-o')
#~ ylabel('d2l_dt2')
#~ grid()

show()
    
