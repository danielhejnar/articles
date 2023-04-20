You are a NASA engineer who uses Matlab for the preliminary design of optimal low-thrust interplanetary rendezvous for space missions. YOu are asked to write the code for the following problem for a low-thrust interplanetary trajectory between Earth and an asteroid using the VCGF method outlined below. Now, You are given the following information, make sure to include the `particleswarm` following the 10 step algorithm procedure, `fzero`, `cost_objective` functions, constants, and everything else. Compute the optimized trajectory using the thrust acceleration equations and plot the orbit bodies and trajectory result. The mission will have the minimum time of flight case. You are going to use the following PSO algorithm, and the following mission values and create the matlab code:

# VCGF Method

The VCGF method requires that a spacecraft fly in a virtual conic orbit and the virtual central gravitational field to satisfy the boundary constraints. There are a few steps in the VCGF method. Firstly, feasible virtual gravity field determined by the parameter set `[mu_vg, r0_vec]` are required, in which the spacecraft runs in virtual conic orbits to satisfy trajectory constraints. Secondly, the required thrust to form the virtual gravity is computed, and the fuel consumption is calculated. Finally, considering the objective function, the PSO algorithm is adopted to find the optimal initial guess for a more accurate optimizer. A virtual central gravitational field can be defined by two parameters: `mu_vg`, the magnitude of virtual gravity; `r0_vec`, the displaced position parameter From the definition of the virtual central gravitational field, we can see that the earth's gravity is a particular case of virtual gravity, whose gravity parameters are `mu_vg` = 1 DU/TU^2 = 0.0000059224 km/s^2, `r0_vec` = 0 DU where the distance unit, 1 DU = 1 (AU) = 149,597,870.70 km and 1 TU = `0.5*pi` year = 58.17 days = 5,025,888 seconds.
### Process of PSO algorithm with VGCF Method

The MATLAB function `particleswarm` is used for the PSO algorithm. In the PSO algorithm, the free parameters are the magnitude of virtual gravity `mu_vg` and the projection of `r0_vec` in the x=axis: `r0x`. The `objective function` can be the fuel cost of the whole trajectory or the flight time. Here we set the maximal number of iterations in the PSO algorithm to `N`. Take an example of orbit rendezvous trajectory design. Given the range of free variables `r0_x` in `[r0x_l, r_0xu]` and `mu_vg` in `[mu_vgl, mu_vgu]`, the initial point A `[ra_vec, va_vec]` and the terminal point B `[rb_vec, vb_vec]`. There is a large amount of parameter set `[r0_vec, mu_vg]`, and each one corresponds to a trajectory, and the feasible trajectories are those that can satisfy boundary constraints. The PSO algorithm is adopted to obtain feasible parameter set`[r0_vec, mu_vg]` from their ranges to determine the virtual gravity and the corresponding virtual conic orbits. There are a few steps in the algorithm. To begin with, the range of free parameters is set, and a large number of possible trajectories, corresponding to the same number of parameter sets `[r0_vec, mu_vg]`, are obtained. Furthermore, those feasible trajectories which can satisfy the constraints are chosen. Finally, the value of the objective function is calculated, and the optimal solution is obtained.

The process of the PSO algorithm using MATLAB's `particleswarm` function is listed as follows:

Step 1: Find the ranges of free variables `mu_vg` in `[mu_vgl, mu_vgu]` and r0_x in `[r0x_l, r0x_u]`;

Step 2: Initialize the particle's position with a uniformly distributed random vector `[r0_x, mu_vg]` in their ranges; Given parameters `ra_vec`, `rb_vec`, `va_vec` it needs to obtain feasibe parameter `mu_vg` to solve equation (2). While in the three dimension problem, both `r0_vec`, `mu_vg` are required to optimize. Given the parameters `ra_vec`, rb_vec, `va_vec` , it needs to obtain feasible parameter `mu_vg` to solve equation (2). The root finding function `fzero` in MATLAB is used to solve this part of the problem.

Step 3: Calculate `r0_y` through equation (4) then the parameter set `[r0_vec, mu_vg]` is obtained;

```matlab
a1 = va_y-vb_y;
b1 = vb_x-va_x;
c1 = (rb_x.*vb_y - rb_y.*vb_x) - (ra_x.*va_y - ra_y.*va_x);
% Equation (4)
r0_y = (c1 - a1*r0_x)/b1;
```

Step 4: Calculate the radius and velocity state of initial point A `[rvgb_vec, vvgb_vec]` and terminal point B `[rvgb_vec, vvgb_vec]` in the virtual gravity through equations (1a-1d) to equation (1d);

```matlab
% Here, in the X-Y plane, we set: 
ra_vec = [ra_x, ra_y];
rb_vec = [rb_x, rb_y];
va_vec =  [va_x, va_y];
vb_vec = [vb_x, vb_y];

% Equation (1)
rvga_vec = ra_vec + r0_vec; % (1a)
vvga_vec = va_vec; % (1b)
rvgb_vec = rb_vec + r0_vec; % (1c)
vvgb_vec = vb_vec; % (1d)
```

Step 5: Calculate transfer angle from point A and point B in virtual gravity field through equation (2a-2e);

```matlab
% Equation (2)
hvg_vec = cross(rvga_vec, vvga_vec) = cross(rvgb_vec, vvgb_vec); % (2a)
delta_f = acos(dot(rvga_vec, rvgb_vec) ./ dot(norm(rvga_vec), norm(rvgb_vec)); % (2b)
r_vga = (hvg_vec.^2./mu_vg) .* (1./(1+e_vgc.*cos(f_vga); % (2c)
f_vgb = f_vga + delta_f; % (2d)
r_vga = (hvg_vec.^2./mu_vg) .* (1./(1+e_vgc.*cos(f_vgb); % (2e)
```

where `hvg_vec`, `e_vg` are the angular of momentum and the eccentricity of the virtual Keplerian orbit, while `f_vgb`, `f_vgb` are the true anomalies of point A and point B in the virtual central gravitational field, respectively. The range of parameter `f_vga` is `[0, 2*pi]`, while `r0_vec` should satisfy `hvg_vec = cross(rvga_vec, vvga_vec) = cross(rvgb_vec, vvgb_vec)`. 

Step 6: Calculate the orbital elements of point A through its radius and velocity in the virtual gravity field, `coe_vgA= [a_vgA, e_vgA, i_vgA, omega_vgA, Omega_vga, TA_vgA]`, here `i_vgA`, `omega_vgA`, `Omega_vgA` are the virtual conic orbital inclination, argument of periapsis, longitude of the ascending node, and true anomaly, respectively;

Step 7 Calculate the true anomaly at point B in the designed virtual gravity 
`TA_vgB` = `TA_vgA` + `delta_theta`;

Step 8 Calculate the radius and velocity [rvgB_vec, vvgB_vec] through the orbital elements `coe_vgB = [a_vgA, e_vgA, i_vgA, omega_vgA, Omega_vgA, TA_vgB]`;

Step 9: Calculate the radius error `delta_eps_r = abs(rvgB_vec - rvgb_vec)` and velocity error `delta_eps_v = abs(vvgB_vec - rvgb_vec)`; if `delta_eps_r <= 10e-4` and `delta_eps_v <= 10e-4`; the trajectory in the virtual gravity can satisfy the boundary constraint. Then record the parameter set `[r0_vec(i), f_vga(i)]` and calculate the fuel consumption (equation 7) or fight time (equation 5) as its fitness. Otherwise, if `deptla_eps_r > 10e-4` and `delta_eps_v > 10e-4`; then set the fitness of the `particle = 1`;

```matlab
% Eqaution (5) is the objective function in the minimum flight time case
% f_t = f(f_vga, r0x_vec)
ft_int = @(f) (sqrt(a_vg^3.*(1-e_vg)^3)/mu_vg).*(1./(1+evg.*cos(f))^2);
f_t = integral(ft_int, f_vga, f_vgb); % (5)

% Equation (6) is the objective function in the minimum energy consumption case
% f_e = f(f_vga, r0x_vec)
F1 = mu/r^2; % Gravitational Force of central body
F2 = mu_vg/r_vg^2; % Gravitational Force of virtual central body
r_vg = (h_vg^2/mu_vg) * (1/(1+e_vg*cos(f_vg))); 
f_e = @(f) integral(F2_vec - F1_vec, f_vga, f_vgb); % (6)

% Equation (7) is the objective function in minimum fuel consumption case
% f_dV = f_min(fvga, r0x_vec) 
fdV_int = @(f) F2 - F1
f_dV = integral(F2 - F1, f_vga, f_vgb); % (7)
```

Note, in this optimization problem, `r0_vec` and `mu_vg` are independent variables. Feasible variables `r0_vSYCec`, `mXu_vg` need to be optimized to minimize the flight time or the fuel consumption. This paper uses the particle swarm optimization (PSO) algorithm to solve it. The detail of the PSO algorithm is stated in the following section.

Step 10: Initialize the particles' best-known position to its initialx position; then update the swarm's best-known position and its fitness; If the number of iterations `i < N`; return to step 4. If the number of iterations `i > N`, finished; It should be noted that, there is a large number of feasible virtual Keplerian orbits that can satisfy boundary constraints and the optimal one is the subcategory of them.

## Mission applications

A spacecraft is transferred from the Earth to rendezvous with the Asteroid. The boundary conditions are listed in the code block below. Here, to analyze the relationship between the transfer angle, the required thrust and the fuel cost, the VCGF method is applied to generate initial guesses in the case of three transfer angles `delta_theta` a half revolution `delta_theta = pi`, one revolution `delta_theta = 2*pi`, and two revolutions `delta_theta = 4*pi`. Using the VCGF method, those feasible trajectories are expressed as virtual conic orbits and parameterized by the parameter set `[r0_vec, mu_2]`. As discussed in the section "Rendezvous trajectory design using VCGF method", for one virtual conic trajectory, the transfer angle is less than `pi` `delta_theta < 2*pi`. While in the case of multi-revolutions condition `delta_theta > 2*pi`, the whole trajectory is divided into a few segments in discrete points, and each segment corresponds to a virtual conic orbit. To testify to the suitability of the proposed method, the results are compared with the five-degree inverse polynomial shaped-based method in terms of magnitude and direction of required thrust, the flight time and the fuel consumption. The boundary of the Earth-Asteroid rendezvous mission is listed in the matlab code block below.

```matlab
% Initial and Final conditions for transfer trajectory.
N = 100;
n = [1, 2, 4];
ra = 1; % (DU)
rb = 1.52; % (DU)
theta_a = 0 % (rad)
theta_b = n*pi % (rad)
ra_dot = 0; % (DU/TU)     
rb_dot = 0; % (DU/TU)      
theta_a_dot = 0.6564; % (rad/TU)
theta_b_dot = 0.5333; % (rad/TU)
```

### Thrust Acceleration

In the geocentric coordinate system, the equation of motion of spacecraft under continuous thrust can be written as `T_ac_vec = d2r_vec/dt2 + mu_vg/norm(r)^3*r_vec`. Here we assume that the thrust can form a virtual gravity, and then in the virtual central gravitational coordinate system, the equation of the spacecraft's motion under continuous thrust can be written as `d2rvg_vec/dt2 + mu_vg/norm(r_vg)^3*rvg_vec = 0`

```matlab
% Equation (11)
rvg_vec = r_vec - r0_vec; % Note: r0_vec is a constant in one virtual gravity, so we have r0_vec' = 0
vvg_vec = v_vec;
```

where `[v_vec, vvg_vec]` are velocity vectors in the geocentric coordinates system and virtual central gravitational coordinate system. Through equations (9), (10), and (11), the required thrust acceleration (TA) for equation (12) is  `T_ac_vec = mu/norm(r_vec)^3.*r_vec - mu_vg/norm(rvg_vec)^3.*rvg_vec`

```matlab
% Equation (13)
rvg_vec = r_vec + r0_vec;
T_ar1 = mu/norm(r_vec)^2;
T_ar2 = mu_vg/norm(rvg_vec)^2;
T_ac = T_ar1*sin(acos(T_ar2/T_ar1));
```

where `T_ar1`, `T_ar2`, `T_ac` are a magnitude of gravity acceleration, virtual central gravitational acceleration, and required true anomoly, `TA`, respectively.

In the case of 3D trajectory design, as shown in Figure 6, `rm_vec` is the position vector of spacecraft at point `M`, and `rvgm_vec` is the position vector at point `M` in the virtual gravity. Assuming TA is `T_ac_vec = [T_acn2, T_actheta2]`, where `T_acn2` is aligned along the radical direction, and `T_actheta2` is aligned along the circulation direction; `T_ar1` is the geocentric gravitation force acceleration, and `T_ar2` is the virtual centric gravitational force acceleration; `alpha` is the angle between `T_actheta2` and `T_ar2`, and `beta` is the angle between `rm1_vec` and `h_2`, then the required `TA` can be computed as shown below in equations (14a-14d)

```matlab
% Equation (14)
beta = acos((norm(dot(rm_vec, hvg_vec)))/dot((norm(rm_vec),norm(hvg_vec))); % (14a)
T_ar1 = mu/norm(rm_vec)^2; % (14b)
T_ar2 = mu_vg/norm(rvgm_vec)^2; % (14c)
alpha= asin(norm(T_ar2)/(T_ar1*sin(beta))); % (14d)
T_actheta2 = T_ar2*sin(beta)*cos(beta); % (14e)
T_acn2 = T_ar1*cos(beta); % (14f)
```
