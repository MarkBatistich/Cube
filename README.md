# Cube

## Dynamic Model

To start, I drew a model of the sytem and defined some variables.

![20220209_165314000_iOS](https://user-images.githubusercontent.com/12192597/153292640-dc4b47b9-1647-48cf-8b72-43c025f5978f.jpg)

$\theta_w$ - angle of the wheel

$\theta_f$ - angle of the frame

$T$ - motor torque

$C_w$ - motor friction torque coefficient

$C_f$ - frame friction torque coefficient

$F_w$ - force of gravity on the wheel

$F_f$ - force of gravity on the frame

$I_c^o$ - moment of inertia of the cube about the origin

$I_w$ - moment of inertia of the wheel about its center of mass

$M_c^o$ - moment on the cube about the origin

$M_w$ - moment of the wheel about its center of mass

$m_w$ - mass of the wheel

$m_f$ - mass of the frame

$m_c$ - mass of the cube

$l_w$ - distance between the wheel's center of mass and the origin

$l_f$ - distance between the frame's center of mass and the origin

$l_c$ - distance between the cube's center of mass and the origin

I also defined the intertial axis such that +x is parellel to an unsloped surface and parallel to the cube face, +y is normal to an unsloped surface in the opposite direction as the force of gravity, and +z is in the direction of counterclockwise (CCW) wheel rotation. The origin O is placed at the corner of the cube touching the surface.

The equations of motion are constructed from the moments.

$M_c^o = I_c^o\ddot{\theta}_f = m_cl_cgsin(\theta_f) - (T - C_w\dot{\theta_w})-C_f\dot{\theta}_f$

$M_w = I_w(\ddot{\theta}_f+\ddot{\theta}_w) = T-C_w\dot{\theta_w}$

The cube moment of inertia about the origin can be found from the parallel axis theorem.

$I_c^o = I_f^o + I_w^o = I_f+m_fl_f^2 + I_w+m_wl_w^2$

The moment equations are solved for angular acceleration

$\ddot{\theta}_f = \frac{m_cl_cgsin(\theta_f)-T+C_w\dot{\theta_w}-C_f\dot{\theta}_f}{I_c^o}$

$\ddot{\theta}_w = \frac{T- C_w\dot{\theta_w}}{I_w} - \ddot{\theta}f$

From these equations, a state space model is made:

$x_1=\theta_f, x_2=\dot{\theta}_f,x_3=\dot{\theta}_w$

$\dot{x}_1 = x_2$

$\dot{x}_2 = \frac{m_cl_cgsin(\theta_f)-T+C_w\dot{\theta_w}-C_f\dot{\theta}_f}{I_c^o}$

$\dot{x}_3 = \frac{T- C_w\dot{\theta_w}}{I_w} - \frac{m_cl_cgsin(\theta_f)-T+C_w\dot{\theta_w}-C_f\dot{\theta}_f}{I_c^o}$

The small angle approximation is applied: $\sin({\theta}_f)={\theta}_f$

The torque is related to the pwm input $u$ by the equation:

$T = K_tu$

where $K_t$ is the motor torque constant.

The state space equations are put into matrix form:

$
\begin{bmatrix}
\dot{x}_1 \cr
\dot{x}_2 \cr
\dot{x}_3 
\end{bmatrix} = 
\begin{bmatrix}
0 & 1 & 0 \cr
\frac{m_cl_cg}{I_c^o} & -\frac{C_f}{I_c^o} & \frac{C_w}{I_c^o} \cr
-\frac{m_cl_cg}{I_c^o} & \frac{C_f}{I_c^o} & -\frac{C_w}{I_w}-\frac{C_w}{I_c^o}
\end{bmatrix}
\begin{bmatrix}
x_1 \cr
x_2 \cr
x_3 
\end{bmatrix} + 
\begin{bmatrix}
0 \cr
-\frac{K_t}{I_c^o} \cr
\frac{K_t}{I_w}-\frac{K_t}{I_c^o}
\end{bmatrix} u $

Now all these values had to be determined. For better scaling, the gcs units were used (grams, centimeters, seconds):

$ g = 981 [\frac{cm}{s^2}]$

$m_c = 710\ [g]$

$l_c = 6.28\ [cm]$

$I_c^o = 37200\ [g*cm^2]$

$I_w = 2740 \ [g*cm^2]$

$C_f = 0$

$C_w = 0$

$K_t = 250000\ [\frac{g*cm^2}{s^2}]$