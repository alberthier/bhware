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
    
def spline4(t ,t1, q, a, b, c, deriv) :
    if (deriv == 2) :
        ret  = a*math.pow(t,2) - ((q + a*math.pow(t1,2))/t1)*t + q
    if (deriv == 1) :
        ret  = (a*math.pow(t,3))/3.0 - ((q + a*math.pow(t1,2))/t1)*(math.pow(t,2)/2.0) + q*t + b
    if (deriv == 0) :
        ret  = (a*math.pow(t,4))/12.0 - ((q + a*math.pow(t1,2))/t1)*(math.pow(t,3)/6.0) + q*(math.pow(t,2)/2.0) + b*t + c
        
    return ret
    
def spline34(t ,t1, n, q, a, b, c, deriv) :
    if (deriv == 2) :
        ret  = a*math.pow(t,n+1) - (q * t)/t1 - a*t1*math.pow(t,n) + q
    if (deriv == 1) :
        ret  = (a*math.pow(t,n+2)) / (n+2.0) - (q * pow(t, 2)) / (2.0*t1) - (a*t1*math.pow(t,n+1)) / (n+1.0) + q * t + b
    if (deriv == 0) :
        ret  = (a*math.pow(t,n+3)) / ((n+2.0)*(n+3.0)) - (q * pow(t, 3)) / (6.0*t1) - (a*t1*math.pow(t,n+2)) / ((n+1.0)*(n+2.0)) + (q * pow(t, 2)) / 2.0 + b * t + c
        
    return ret
        
def trajectoire_sp34(t, t1, n, qx, qy, ax, ay, bx, by, x0, y0, theta_seg, deriv) :
    x = spline34(t, t1, n, qx, ax, bx, x0, deriv)
    y = spline34(t, t1, n, qy, ay, by, y0, deriv)
    if deriv == 0 :
        [x_r, y_r] = rotation_coord(theta_seg, x-x0, y-y0)
        x_r = x0 + x_r
        y_r = y0 + y_r
    else :
        [x_r, y_r] = rotation_coord(theta_seg, x, y)
    
    return [x_r, y_r]
    
def trajectoire_sp3(t, t1, ax, ay, bx, by, x0, y0, deriv) :
    x = spline3(t, t1, 0, ax, bx, x0, deriv)
    y = spline3(t, t1, 0, ay, by, y0, deriv)
    
    return [x, y]
    
def trajectoire_sp4(t, t1, qx, qy, ax, ay, bx, by, x0, y0, deriv) :
    x = spline4(t, t1, qx, ax, bx, x0, deriv)
    y = spline4(t, t1, qy, ay, by, y0, deriv)
    
    return [x, y]
    
def trajectoire_rotation(t, t1, x_r, y_r, R, theta0, angle) :
    theta = theta0 + (t * angle) / t1
    x = x_r + R * cos(theta)
    y = y_r + R * sin(theta)
    
    return [x, y]
    
def rotation_config(t1, ax, ay, bx, by, x1, y1, angle) :
    pos_f = trajectoire_sp3(t1, t1, ax, ay, bx, by, x1, y1, 0)
    Diff1BS_f = trajectoire_sp3(t1, t1, ax, ay, bx, by, x1, y1, 1)
    Rinv_sp3 = Rinv_courbure(t1, t1, 0, 0.0, 0.0, ax, ay, bx, by, x1, y1, 0, 'sp3')
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
    
    return [x_cr1, y_cr1, fabs(Rinv_sp3), signe_Rinv_sp3, theta0_r1, pos_fr, theta_fr, qx, qy]
    
def Rinv_courbure(t, t1, n, qx, qy, ax, ay, bx, by, x0, y0, theta_seg, sp) :
    if sp == 'sp34' :
        diff1BS = trajectoire_sp34(t, t1, n, qx, qy, ax, ay, bx, by, x0, y0, theta_seg, 1)
        diff2BS = trajectoire_sp34(t, t1, n, qx, qy, ax, ay, bx, by, x0, y0, theta_seg, 2)
    if sp == 'sp4' :
        diff1BS = trajectoire_sp4(t, t1, qx, qy, ax, ay, bx, by, x0, y0, 1)
        diff2BS = trajectoire_sp4(t, t1, qx, qy, ax, ay, bx, by, x0, y0, 2)
    if sp == 'sp3' :
        diff1BS = trajectoire_sp3(t, t1, ax, ay, bx, by, x0, y0, 1)
        diff2BS = trajectoire_sp3(t, t1, ax, ay, bx, by, x0, y0, 2)

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

def s3_ab(x0, y0, theta0, x1, y1, theta1, t1) :
    bx = (3.0 / (2.0*t1)) * ((x1 - x0 - y1 + y0) / (tan(theta1) - tan(theta0)))
    print("test : {0}".format((tan(theta1) - tan(theta0))))
    print("bx : {0}".format(bx))
    by = bx * tan(theta0)
    ax = (6.0 / pow(t1, 2)) *(-t1*bx + x1 - x0)
    ay = (6.0 / pow(t1, 2)) *(-t1*tan(theta0)*bx + y1 - y0)
    
    return ([ax, ay, bx, by])

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

def sds_ab(x1, y1, theta1, x2, y2, theta2, t1, n1, n2, qx1, qy1, qx2, qy2) :
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
    
    return ([ax1, ay1, bx1, by1, ax2, ay2, bx2, by2, qx1, qy1, qx2, qy2, theta_seg])

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
    print("S1 : "+str(S1))
        
    S2 = 1.0
    if theta2 < 0.0 :
        S2 = -S2
    print("S2 : "+str(S2))

    
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

def s4s4_ab(x1, y1, theta1, x2, y2, theta2_r, t1, qx1, qy1, qx2, qy2) :
    theta2 = theta2_r + math.pi
    if theta2 > math.pi :
        theta2 = theta2 - 2.0*math.pi
    
    theta_seg1 = math.atan2(y2 - y1, x2 - x1)
    theta_seg2 = math.atan2(y1 - y2, x1 - x2)
    D = sqrt(pow(x2-x1,2) + pow(y2-y1,2))
    Cmax = 0.1
    if 0.4*D > Cmax :
        C = Cmax
    else :
        C = 0.4*D 
    #~ k = 0.57
    
    delta_theta1 = theta1 - theta_seg1
    if delta_theta1 < -math.pi :
        delta_theta1 = delta_theta1 + 2*math.pi
    if delta_theta1 > math.pi :
        delta_theta1 = delta_theta1 - 2*math.pi
        
    sCy1 = 1.0
    if delta_theta1 < 0.0 :
        sCy1 = -sCy1
        
    delta_theta2 = theta2 - theta_seg2
    if delta_theta2 < -math.pi :
        delta_theta2 = delta_theta2 + 2*math.pi
    if delta_theta2 > math.pi :
        delta_theta2 = delta_theta2 - 2*math.pi
        
    sCy2 = 1.0
    if delta_theta2 < 0.0 :
        sCy2 = -sCy2
        
    cos_theta_seg1 = cos(theta_seg1) - sin(theta_seg1)*sCy1
    sin_theta_seg1 = sin(theta_seg1) + cos(theta_seg1)*sCy1
    cos_theta_seg2 = cos(theta_seg2) - sin(theta_seg2)*sCy2
    sin_theta_seg2 = sin(theta_seg2) + cos(theta_seg2)*sCy2
        
    #~ print("Cy :")
    #~ print(Cy)
    
    Tx1 = (t1/2.0) * qx1
    Tx2 = (t1/2.0) * qx2
    Ty1 = (t1/2.0) * qy1
    Ty2 = (t1/2.0) * qy2
    T2x1 = ((t1*t1)/3.0) * qx1
    T2x2 = ((t1*t1)/3.0) * qx2
    T2y1 = ((t1*t1)/3.0) * qy1
    T2y2 = ((t1*t1)/3.0) * qy2    
    A3 = -pow(t1,3)/6.0
    A4 = -pow(t1,4)/12.0
    A01 = ((t1*t1)/3.0) * (qx1 * cos_theta_seg1 + qy1 * sin_theta_seg1)
    A02 = ((t1*t1)/3.0) * (qx2 * cos_theta_seg2 + qy2 * sin_theta_seg2) - (A4/A3) * (cos_theta_seg2*(Tx1+Tx2) + sin_theta_seg2*(Ty1+Ty2))
    A11 = t1 * (cos_theta_seg1 + tan(theta1) * sin_theta_seg1)
    A12 = -(A4/A3) * (cos_theta_seg2 + sin_theta_seg2 * tan(theta1))
    A13 = (cos_theta_seg2 + tan(theta2) * sin_theta_seg2) * (t1 - (A4/A3))
    E0 = Tx1
    E1 = (A4/A3)*(Tx1 + Tx2) + (T2x1 - T2x2) + (x1 - x2)
    E2 = (A4/A3) + t1
    E3 = (A4/A3) - t1
    E4 = Ty1
    E5 = (A4/A3)*(Ty1 + Ty2) + (T2y1 - T2y2) + (y1 - y2)
    E6 = tan(theta1) * ((A4/A3) + t1)
    E7 = tan(theta2) * ((A4/A3) - t1)
    E8 = -A4 * cos_theta_seg1 / A11
    #~ E8 = -A4 / A11
    E9 = -A4 * sin_theta_seg1 / A11
    #~ E9 = -A4 / A11
    E10 = (C - A01) / A11
    E11 = (A4/A13) * (cos_theta_seg2 + (A12/A11) * cos_theta_seg1)
    #~ E11 = (A4/A13) * (1.0 + (A12/A11))
    E12 = (A4/A13) * (sin_theta_seg2 + (A12/A11) * sin_theta_seg1)
    #~ E12 = (A4/A13) * (1.0 + (A12/A11))
    E13 = (C - A02 -(A12/A11)*(C - A01)) / A13
    E14 = A3 + E8
    E15 = 2.0*A4 + E2*E8 + E3*E11
    E16 = E2*E9 + E3*E12
    E17 = E0 + E10
    E18 = E2*E10 + E3*E13 + E1
    E19 = E8 * tan(theta1)
    E20 = E6*E8 + E7*E11
    E21 = A3 + E9*tan(theta1)
    E22 = 2.0*A4 + E6*E9 + E7*E12
    E23 = E10*tan(theta1) + E4
    E24 = E6*E10 + E7*E13 + E5
    
    #~ E25 = t1*tan(theta1)*E8*cos(theta_seg1) - (A4 + t1*E8)*sin(theta_seg1)
    #~ E26 = (A4 + t1*tan(theta1)*E9)*cos(theta_seg1) - t1*E9*sin(theta_seg1)
    #~ E27 = (t1*E10 + T2x1)*(-sin(theta_seg1)) + (t1*tan(theta1)*E10 + T2y1)*cos(theta_seg1)
    #~ E28 = E14 - (E25/E26)*E9
    #~ E29 = E15 - (E25/E26)*E16
    #~ E30 = ((Cy - E27)/E26)*E9 + E17
    #~ E31 = ((Cy - E27)/E26)*E16 + E18
    #~ E32 = E19 - (E25/E26)*E21
    #~ E33 = E20 - (E25/E26)*E22
    #~ E34 = ((Cy - E27)/E26)*E21 + E23
    #~ E35 = ((Cy - E27)/E26)*E22 + E24
    #~ E36 = E35*E29 - E33*E31
    #~ E37 = E35*E28 + E34*E29 - E32*E31 - E33*E30
    #~ E38 = E34*E28 - E32*E30
    #~ 
    #~ delta_k = pow(E37,2) - 4.0*E36*E38
    #~ 
    #~ k1 = (-E37 + sqrt(delta_k)) / (2.0*E36)
    #~ k2 = (-E37 - sqrt(delta_k)) / (2.0*E36)
    #~ if k1 > 0.0 :
        #~ k = k1
    #~ else :
        #~ k = k2
    #~ print("k :")
    #~ print(k)
    
    A5 = cos(theta_seg1) / cos(theta1)
    E40 = -(A3 + E8*(1.0 - A5)) / (E9*(1.0 - A5))
    E41 = -(Tx1 + E10*(1.0 - A5)) / (E9*(1.0 - A5))
    E42 = E9*E41 + E17
    E43 = E16*E41 + E18
    E44 = E9*E40 + E14
    E45 = E16*E40 + E15
    E46 = E21*E41 + E23
    E47 = E22*E41 + E24
    E48 = E21*E40 + E19
    E49 = E22*E40 + E20
    E50 = E43*E49 - E45*E47
    E51 = E42*E49 + E43*E48 - E44*E47 - E45*E46
    E52 = E42*E48 - E44*E46
    
    delta_k = pow(E51,2) - 4.0*E50*E52
    
    k1 = (-E51 + sqrt(delta_k)) / (2.0*E50)
    k2 = (-E51 - sqrt(delta_k)) / (2.0*E50)
    if k1 > 0.0 :
        k = k1
    else :
        k = k2
    #~ print("k :")
    #~ print(k)
    
    ay1 = ( (((E19 + k*E20)*(E17 + k*E18)) / ((E21 + k*E22)*(E14 + k*E15))) - (E23 + k*E24) / (E21 + k*E22) ) / (1.0 - (((E19 + k*E20)*(E9 + k*E16)) / ((E21 + k*E22)*(E14 + k*E15))) )
    
    ax1 = -((E9 + k*E16) / (E14 + k*E15)) * ay1 - ((E17 + k*E18) / (E14 + k*E15))
    
    bx2 = E11*ax1 + E12*ay1 + E13
    
    bx1 = E8*ax1 + E9*ay1 + E10
    
    ax2 = -ax1 - bx1/A3 - bx2/A3 - (Tx1 + Tx2) /A3
    
    ay2 = -ay1 - (tan(theta1)/A3)*bx1  - (tan(theta2)/A3)*bx2 - (Ty1 + Ty2) /A3
    
    by1 = bx1 * tan(theta1)
    by2 = bx2 * tan(theta2)
    
    #~ print("eq.1")
    #~ print(A4*cos_theta_seg1*ax1 + A4*sin_theta_seg1*ay1 + A11*bx1 + A01 - C)
    #~ print("eq.2")
    #~ print(-A4*cos_theta_seg2*ax1 - A4*sin_theta_seg2*ay1 + A12*bx1 + A13*bx2 + A02 - C)
    print("x_p1(t1)+x_p2(t1) : {0}".format( (A3*ax1 + (t1/2.0)*qx1 + bx1) + (A3*ax2 + (t1/2.0)*qx2 + bx2) ))
    print("y_p1(t1)+y_p2(t1) : {0}".format( (A3*ay1 + (t1/2.0)*qy1 + by1) + (A3*ay2 + (t1/2.0)*qy2 + by2) ))
    
    return ([ax1, ay1, bx1, by1, ax2, ay2, bx2, by2])

def eq_by(bx, theta) :
    return (bx * math.tan(theta))


def equations_oneSegTraj_simple(p, x1, y1, theta1, theta_seg1, x2, y2, theta2, theta_seg2, t1, C):
    ax1, ay1, bx1, ax2, ay2, bx2, k = p
    #~ x1, y1, theta1, theta_seg1, x2, y2, theta2, theta_seg2, t1, C = args
        
    eqs = ( (spline4(t1 ,t1, 0.0, ax1, bx1, x1, 0) - spline4(0.0 ,t1, 0.0, ax1, bx1, x1, 0)) * math.cos(theta_seg1) + (spline4(t1 ,t1, 0.0, ay1, eq_by(bx1, theta1), y1, 0) - spline4(0.0 ,t1, 0.0, ay1, eq_by(bx1, theta1), y1, 0)) * math.sin(theta_seg1) - C
        , (spline4(t1 ,t1, 0.0, ax2, bx2, x2, 0) - spline4(0.0 ,t1, 0.0, ax2, bx2, x2, 0)) * math.cos(theta_seg2) + (spline4(t1 ,t1, 0.0, ay2, eq_by(bx2, theta2), y2, 0) - spline4(0.0 ,t1, 0.0, ay2, eq_by(bx2, theta2), y2, 0)) * math.sin(theta_seg2) - C
        , spline4(t1 ,t1, 0.0, ax2, bx2, x2, 1) + spline4(t1 ,t1, 0.0, ax1, bx1, x1, 1)
        , spline4(t1 ,t1, 0.0, ay2, eq_by(bx2, theta2), y2, 1) + spline4(t1 ,t1, 0.0, ay1, eq_by(bx1, theta1), y1, 1)
        , spline4(t1 ,t1, 0.0, ax1, bx1, x1, 1) - math.fabs(k) * (spline4(t1 ,t1, 0.0, ax2, bx2, x2, 0) - spline4(t1 ,t1, 0.0, ax1, bx1, x1, 0))
        , spline4(t1 ,t1, 0.0, ay1, eq_by(bx1, theta1), y1, 1) - math.fabs(k) * (spline4(t1 ,t1, 0.0, ay2, eq_by(bx2, theta2), y2, 0) - spline4(t1 ,t1, 0.0, ay1, eq_by(bx1, theta1), y1, 0))
        , ax2 * ay1 - ay2 * ax1
        )
    return eqs
    
def solve_oneSegTraj_simple(x1, y1, theta1, x2, y2, theta2, t1) :
    ## traj = spline 4 - droite - spline 4
    
    theta2b = theta2 + math.pi
    if theta2b > math.pi :
        theta2b = theta2b - 2.0*math.pi
    
    theta_seg1 = math.atan2(y2 - y1, x2 - x1)
    theta_seg2 = math.atan2(y1 - y2, x1 - x2)
    C = 0.2 * math.sqrt(math.pow(x2-x1,2) + math.pow(y2-y1,2))
    args = x1, y1, theta1, theta_seg1, x2, y2, theta2b, theta_seg2, t1, C
    
    sol =  fsolve(equations_oneSegTraj_simple, (1, 1, 1, 1, 1, 1, 1), args=args)
    #~ (1, 1, 1, 1, 1, 1, 1)
    return sol




for n in range(1) :
    x1 = 0.0
    y1 = 0.0
    theta1 = pi*(0.2)
    x2 = 0.3
    y2 = 0.7 #0.65*0.8
    theta2 = pi*(0.6)
    
    #config
    t1 = 0.1
    vmax = 0.6
    EcartRoue = 0.17
    Rinv_ref = 1.0 / (EcartRoue/2.0)
    v_Rinv_ref = vitesse_limite(vmax, EcartRoue, Rinv_ref)
    Db = 0.03 #0.05
    Da = 0.0018 #0.006
    
    #~ sol = s4s4_ab(x1, y1, theta1, x2, y2, theta2, t1, 0.0, 0.0, 0.0, 0.0)
    sol = sds_ab(x1, y1, theta1, x2, y2, theta2, t1, 1, 1, 0.0, 0.0, 0.0, 0.0)
    [ax1, ay1, bx1, by1, ax2, ay2, bx2, by2, qx1, qy1, qx2, qy2, theta_seg] = sol
    print(sol)
    #~ sys.exit(2)

    Rinv_sp4_1 = Rinv_courbure(t1/2.0, t1, 1, 0.0, 0.0, ax1, ay1, bx1, by1, x1, y1, theta_seg, 'sp34')
    v_Rinv_sp4_1 = vitesse_limite(vmax, EcartRoue, fabs(Rinv_sp4_1))
    print("v_Rinv_sp4_1 : {0} m/s (ref : {1} m/s)".format(v_Rinv_sp4_1, v_Rinv_ref))
    if v_Rinv_sp4_1 < v_Rinv_ref :
        flag_angle_rot1 = False
        memo_flag_angle_rot1 = False
        angle_r1 = 0.0
    else :
        flag_angle_rot1 = True
        memo_flag_angle_rot1 = True
        angle_r1 = 0.0
    
    Rinv_sp4_2 = Rinv_courbure(t1/2.0, t1, 1, 0.0, 0.0, ax2, ay2, bx2, by2, x2, y2, theta_seg, 'sp34')
    v_Rinv_sp4_2 = vitesse_limite(vmax, EcartRoue, fabs(Rinv_sp4_2))
    print("v_Rinv_sp4_2 : {0} m/s (ref : {1} m/s)".format(v_Rinv_sp4_2, v_Rinv_ref))
    if v_Rinv_sp4_2 < v_Rinv_ref :
        flag_angle_rot2 = False
        memo_flag_angle_rot2 = False
        angle_r2 = 0.0
    else :
        flag_angle_rot2 = True
        memo_flag_angle_rot2 = True
        angle_r2 = 0.0

    print("memo_flag_angle_rot1 : {0}".format(memo_flag_angle_rot1))
    print("memo_flag_angle_rot2 : {0}".format(memo_flag_angle_rot2))

    qx1_n = 0.0
    qy1_n = 0.0
    x1_n = x1
    y1_n = y1
    theta1_n = theta1
    
    qx2_n = 0.0
    qy2_n = 0.0
    x2_n = x2
    y2_n = y2
    theta2_n = theta2

    while flag_angle_rot1 == False or flag_angle_rot2 == False :
        if flag_angle_rot1 == False :
        ### rotation 1 ###
            sol = s3r_ab(x1, y1, theta1, x2, y2, Db, Da, t1)
            [ax_sp3_1, ay_sp3_1, bx_sp3_1, by_sp3_1] = sol
            #~ print("solution sp3 :")
            #~ print(sol)
            #~ angle_r1 = -pi*0.07
            [x_cr1, y_cr1, Rinv_r1, signe_Rinv_r1, theta0_r1, pos_f1, theta_f1, qx1_n, qy1_n] = rotation_config(t1, ax_sp3_1, ay_sp3_1, bx_sp3_1, by_sp3_1, x1, y1, angle_r1)
            x1_n = pos_f1[0]
            y1_n = pos_f1[1]
            theta1_n = theta_f1
        
        if flag_angle_rot2 == False :
            ### rotation 2 ###
            sol = s3r_ab(x2, y2, modulo_angle(theta2 + pi), x1, y1, Db, Da, t1)
            [ax_sp3_2, ay_sp3_2, bx_sp3_2, by_sp3_2] = sol
            #~ print("solution sp3 :")
            #~ print(sol)
            #~ angle_r2 = -pi*0.07
            #~ angle_r2 = angle_r1
            [x_cr2, y_cr2, Rinv_r2, signe_Rinv_r2, theta0_r2, pos_f2, theta_f2, qx2_n, qy2_n] = rotation_config(t1, ax_sp3_2, ay_sp3_2, bx_sp3_2, by_sp3_2, x2, y2, angle_r2)
            x2_n = pos_f2[0]
            y2_n = pos_f2[1]
            theta2_n = modulo_angle(theta_f2 + pi)
        
        ### nouvelle s4s4 ###
        sol = sds_ab(x1_n, y1_n, theta1_n, x2_n, y2_n, theta2_n, t1, 1, 1, qx1_n, qy1_n, qx2_n, qy2_n)
        [ax1_n, ay1_n, bx1_n, by1_n, ax2_n, ay2_n, bx2_n, by2_n, qx1_n, qy1_n, qx2_n, qy2_n, theta_seg_n] = sol
        
        Rinv_sp4_1_n_0 = Rinv_courbure(0, t1, 1, qx1_n, qy1_n, ax1_n, ay1_n, bx1_n, by1_n, x1_n, y1_n, theta_seg_n, 'sp34')
        Rinv_sp4_1_n_1 = Rinv_courbure(0.01*t1, t1, 1, qx1_n, qy1_n, ax1_n, ay1_n, bx1_n, by1_n, x1_n, y1_n, theta_seg_n, 'sp34')
        dRinv_sp4_1_n = fabs(Rinv_sp4_1_n_1) - fabs(Rinv_sp4_1_n_0)
        
        Rinv_sp4_2_n_0 = Rinv_courbure(0, t1, 1, qx2_n, qy2_n, ax2_n, ay2_n, bx2_n, by2_n, x2_n, y2_n, theta_seg_n, 'sp34')
        Rinv_sp4_2_n_1 = Rinv_courbure(0.01*t1, t1, 1, qx2_n, qy2_n, ax2_n, ay2_n, bx2_n, by2_n, x2_n, y2_n, theta_seg_n, 'sp34')
        dRinv_sp4_2_n = fabs(Rinv_sp4_2_n_1) - fabs(Rinv_sp4_2_n_0)
        
        if flag_angle_rot1 == False :
            if dRinv_sp4_1_n < 0.0 :
                flag_angle_rot1 = True
            else :
                angle_r1 = angle_r1 + signe_Rinv_r1 * 0.02*pi
            
        if flag_angle_rot2 == False :
            if dRinv_sp4_2_n < 0.0 :
                flag_angle_rot2 = True
            else :
                angle_r2 = angle_r2 + signe_Rinv_r2 * 0.02*pi
            
    print("angle_r1 : {0}".format(angle_r1/pi))
    print("angle_r2 : {0}".format(angle_r2/pi))
    
    #~ qx1_n = 0.0
    #~ qy1_n = 0.0
    #~ qx2_n = 0.0
    #~ qy2_n = 0.0
    #~ sol = sds_ab(x1_n, y1_n, theta1_n, x2_n, y2_n, theta2_n, t1, 1, 1, qx1_n, qy1_n, qx2_n, qy2_n)
    #~ [ax1_n, ay1_n, bx1_n, by1_n, ax2_n, ay2_n, bx2_n, by2_n, theta_seg_n] = sol
    
    N = 30
    l_t = [(k/(N*1.0))*t1 for k in range(N+1)]
    
    
    ### plots ###
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
        [x_1, y_1] = trajectoire_sp34(t, t1, 1, 0.0, 0.0, ax1, ay1, bx1, by1, x1, y1, theta_seg,0)
        l_x_1.append(x_1)
        l_y_1.append(y_1)
        [x_2, y_2] = trajectoire_sp34(t, t1, 1, 0.0, 0.0, ax2, ay2, bx2, by2, x2, y2, theta_seg, 0)
        l_x_2.append(x_2)
        l_y_2.append(y_2)
        
        if memo_flag_angle_rot1 == False :
            [x_sp3_1, y_sp3_1] = trajectoire_sp3(t, t1, ax_sp3_1, ay_sp3_1, bx_sp3_1, by_sp3_1, x1, y1, 0)
            l_x_sp3_1.append(x_sp3_1)
            l_y_sp3_1.append(y_sp3_1)
            [x_r1, y_r1] = trajectoire_rotation(t, t1, x_cr1, y_cr1, 1.0/Rinv_r1, theta0_r1, angle_r1)
            l_x_r1.append(x_r1)
            l_y_r1.append(y_r1)
        
        if memo_flag_angle_rot2 == False :
            [x_sp3_2, y_sp3_2] = trajectoire_sp3(t, t1, ax_sp3_2, ay_sp3_2, bx_sp3_2, by_sp3_2, x2, y2, 0)
            l_x_sp3_2.append(x_sp3_2)
            l_y_sp3_2.append(y_sp3_2)
            [x_r2, y_r2] = trajectoire_rotation(t, t1, x_cr2, y_cr2, 1.0/Rinv_r2, theta0_r2, angle_r2)
            l_x_r2.append(x_r2)
            l_y_r2.append(y_r2)
        
        if (memo_flag_angle_rot1 == False) or (memo_flag_angle_rot2 == False) :
            [x_sp4_n1, y_sp4_n1] = trajectoire_sp34(t, t1, 1, qx1_n, qy1_n, ax1_n, ay1_n, bx1_n, by1_n, x1_n, y1_n, theta_seg_n,0)
            l_x_sp4_n1.append(x_sp4_n1)
            l_y_sp4_n1.append(y_sp4_n1)
            [x_sp4_n2, y_sp4_n2] = trajectoire_sp34(t, t1, 1, qx2_n, qy2_n, ax2_n, ay2_n, bx2_n, by2_n, x2_n, y2_n, theta_seg_n, 0)
            l_x_sp4_n2.append(x_sp4_n2)
            l_y_sp4_n2.append(y_sp4_n2)
        
        l_Rinv_c.append(Rinv_courbure(t, t1, 1, 0.0, 0.0, ax1, ay1, bx1, by1, x1, y1, theta_seg, 'sp34'))
        l_v_lim.append(vitesse_limite(0.6, 0.17, l_Rinv_c[-1]))
        l_v_lim_s2.append(vitesse_limite(0.6, 0.17, Rinv_courbure(t1-t, t1, 1, 0.0, 0.0, ax2, ay2, bx2, by2, x2, y2, theta_seg, 'sp34')))
        if memo_flag_angle_rot1 == False :
            l_v_lim_r.append(vitesse_limite(0.6, 0.17, Rinv_courbure(t, t1, 0, 0.0, 0.0, ax_sp3_1, ay_sp3_1, bx_sp3_1, by_sp3_1, x1, y1, 0, 'sp3')))
        if memo_flag_angle_rot2 == False :
            l_v_lim_r2.append(vitesse_limite(vmax, 0.17, Rinv_courbure(t1-t, t1, 0, 0.0, 0.0, ax_sp3_2, ay_sp3_2, bx_sp3_2, by_sp3_2, x2, y2, 0, 'sp3')))
        if (memo_flag_angle_rot1 == False) or (memo_flag_angle_rot2 == False) :
            l_v_lim_s4_n1.append(vitesse_limite(vmax, EcartRoue, Rinv_courbure(t, t1, 1, qx1_n, qy1_n, ax1_n, ay1_n, bx1_n, by1_n, x1_n, y1_n, theta_seg_n, 'sp34')))
            l_v_lim_s4_n2.append(vitesse_limite(vmax, EcartRoue, Rinv_courbure(t1-t, t1, 1, qx2_n, qy2_n, ax2_n, ay2_n, bx2_n, by2_n, x2_n, y2_n, theta_seg_n, 'sp34')))
        
    
    figure(1)
    plot(l_x_1, l_y_1, 'o')
    plot(l_x_2, l_y_2, 'o')
    plot([l_x_1[-1], l_x_2[-1]], [l_y_1[-1], l_y_2[-1]], '-r')
    
    if memo_flag_angle_rot1 == False :
        plot(l_x_sp3_1, l_y_sp3_1, 'o')
        #centre de rotation 1
        plot(x_cr1, y_cr1, 'or')
        plot(l_x_r1, l_y_r1, 'o')
        plot(l_x_r1[0], l_y_r1[0], 'oy')
    
    if memo_flag_angle_rot2 == False :
        plot(l_x_sp3_2, l_y_sp3_2, 'o')
        #centre de rotation 2
        plot(x_cr2, y_cr2, 'or')
        plot(l_x_r2, l_y_r2, 'o')
        plot(l_x_r2[0], l_y_r2[0], 'oy')
    
    if (memo_flag_angle_rot1 == False) or (memo_flag_angle_rot2 == False) :
        plot(l_x_sp4_n1, l_y_sp4_n1, 'o')
        plot(l_x_sp4_n2, l_y_sp4_n2, 'o')
        plot([l_x_sp4_n1[-1], l_x_sp4_n2[-1]], [l_y_sp4_n1[-1], l_y_sp4_n2[-1]], '-r')
    
    grid()
    xlim(-0.1, 0.8)
    ylim(-0.1, 0.8)
    
    
    figure(2)
    #~ plot(l_Rinv_c, '-o')
    l_v_lim.extend(l_v_lim_s2)
    l_v_lim.extend([0.0])
    if memo_flag_angle_rot1 == False :
        l_v_lim.extend(l_v_lim_r)
        l_v_lim.extend([0.0])
    if (memo_flag_angle_rot1 == False) or (memo_flag_angle_rot2 == False) :
        l_v_lim.extend(l_v_lim_s4_n1)
        l_v_lim.extend([0.0])
        l_v_lim.extend(l_v_lim_s4_n2)
    if memo_flag_angle_rot2 == False :
        l_v_lim.extend([0.0])
        l_v_lim.extend(l_v_lim_r2)
    plot(l_v_lim, '-o')
    vmin = vitesse_limite(vmax, EcartRoue, 1.0/(EcartRoue/2.0))
    plot([0, len(l_v_lim)], [vmin, vmin], '-k')
    ylim(0.0, vmax+0.02)
    
    
grid()
show()
    
