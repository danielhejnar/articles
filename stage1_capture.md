You are a NASA engineer who uses Matlab for the preliminary design of optimal low-thrust interplanetary rendezvous for space missions. Please also include the code for any user-defined functions used. Your task is to create a Matlab program for the stage 1 asteroid capture portion using the following procedure outlined below: 

# Capturing Near-Earth Asteroids (NEAs) around Earth

## Introduction

### Capture of an Asteroid into Stable Earth Orbit

Significant work has been done to deter and deflect hazardous asteroids and other spatial bodies (e.g., [27]). On the other hand, our work is part of a growing body of literature (e.g., [11,12]) to consider the energetics and acceleration requirements to capture an asteroid into a stable orbit around the Earth. The present work focuses on using 2-body and 3-body dynamics, along with lowthrust acceleration or impulsive `delta_V` maneuvers, to capture an NEA into a stable orbit around the Earth. In order to do so, an acceleration term has been introduced to the 2-body and 3-body equations of motion. This capture acceleration works to maneuver the asteroid until it enters the Earth's sphere of influence. Upon entering the sphere of influence of the Earth, two different methods of capturing the asteroid have been studied. First, an impulsive force method is studied, which provides an instantaneous change in velocity required for capture. The second method approximates the continuous acceleration required over a period of time to capture the asteroid. The only fixed parameter in the study is selected to be the maximum time required from rendezvous to capture. Based on the general principle of a timely return on an investment, the maximum capture time is set to be 10 years.

## Mathematical model

Transfer trajectories from asteroids' current orbits to Earth rendezvous are calculated in an ephemeris model (including the location of the Sun, Earth and Moon), given the initial conditions at the time of perihelion of the asteroid of interest. Due to the purely conceptual nature of the work, certain approximations are made. Firstly, a two-dimensional system where only the components of the orbit that are in the reference plane are considered, thereby neglecting the third dimension, but since `delta_inc` is less than 10 degrees, the planar approximation was considered reasonable as a first-order approximation. The other major approximation is the number of bodies that are considered in the model. When the asteroids are far from the Earth, the model is set up as a two-body problem consisting of the Sun and the asteroid. When the asteroid approaches the Earth's sphere of influence, the model is changed to a three-body problem to include the Earth's influence. This is related to the standard 'patched-conics' assumption [32]. With the masses and relative speeds involved in the problem, this approximation is close enough to introduce minimal error, and we need not consider the more accurate, but more complex, patched three-body approximation [33].

Newton's second law is used to determine the equation of motion of the asteroid effected by the bodies of interest, given at their ephemeris-based locations. Considering both the `x` and `y` components in a Sun-centered inertial frame, the differential equations of motion in first-order form are shown below in equations (1a-1d)

```matlab
% Equations of Motion in the heliocentric reference frame (Sun-centered) when the asteroid is outside the Earth's sphere of influence
dx = vx; % Equation (1a)
dy = vy; % Equation (1b)
dvx = -mu_s*(x^2+y^2)^(-3/2)*x + ax; % Equation (1c)
dvy = -mu_s*(x^2+y^2)^(-3/2)*y + ay; % Equation (1d)
```

where `x` and `y` indicate the position of the asteroid in a heliocentric reference frame and `mu_s=G*M`, where $G$ is the gravitational constant and $M_{s}$ is the mass of the Sun. The variables `ax` and `ay` are functions of position that represent the acceleration profile that is applied to the asteroid. These equations can be solved with initial conditions for both the `x` and `y` components of position and velocity of the asteroid. To obtain Cartesian position and velocity components from orbital elements, we use the usual transformations (see, e.g., [32]).

When the three-body problem is necessary the equations of motion for the Earth are the same as the equations for the asteroid in low-thrust. The equations of motion for the asteroid including the effects of the Earth's gravitation are shown below in equations (2a-2d)

```matlab
% Equations of Motion in the geocentric reference frame (Earth-centered) when the asteroid is within the Earth's sphere of influence
% Equation (2)
dx = vx; % Equation (2a)
dy = vy; % Equation (2b)
dvx = -mu_s*(x^2+y^2)^(-3/2)*x + ax...
     - mu_e*((x-xE)^2+(y-yE)^2)^(-3/2)*(x-xE); % Equation (2c)
dvy = -mu_s*(x^2+y^2)^(-3/2)*y + ay...
     - mu_e*((x-xE)^2+(y-yE)^2)^(-3/2)*(y-yE); % Equation (2d)
```

where `xE`, `yE` indicates the position of the Earth. Our concern will be to estimate the magnitude of the acceleration profile a(t) = ||a(t)|| required to capture asteroids within 10 years.

## Order of magnitude approximation

A method for approximating the thrust acceleration is developed, which assumes near-circular, co-planar orbits, a constant rate of radial velocity, and only considers the gravitational force of the Sun (see Appendix A). This method of approximation is based on energy balance. The resulting acceleration is

```matlab
a_circ = sqrt(mu_s)/tf*(1/ri-1/rf)*(rf-ri)/(4*(sqrt(rf)-sqrt(ri))); % Equation (3)
```

where `ri` and `rf` are the initial and final semimajor axes, respectively, and we considered the case `rf > ri`.

We note that the Edelbaum approximation for low-thrust orbits [34] can also be used to formulate an order of magnitude approximation of the thrust acceleration required to capture the asteroids. We discuss this approximation in Appendix B. This approximation is also valid only for near-circular orbits, but assumes constant acceleration, constant thrust yaw angle, and takes account of orbital inclination,

```matlab
a_edel = sqrt(mu_s)/tf*sqrt(1/ri+1/rf-2*cos(pi/2*delta_inc)/sqrt(rf*ri)); % Equation (4)
```

where `delta_inc` is the change in inclination, `delta_i = incf-inci`, and `incf` and `inci` are the final and initial inclinations, respectively. The Edelbaum approximation is more accurate than the circular orbit approximation (3), but it is still only an approximation. The Edelbaum approximation is used in the numerical integrations described in the next section.
which is correct, ignoring factors of order 2 or higher in the eccentricities of the initial and final orbit. For coplanar orbits, `delta_inc = 0;`, and this reduces to

```matlab
a_edel = sqrt(mu_s)/tf*(1/sqrt(ri)-1/sqrt(rf)); % Equation (5)
```

Note that both (3) and (5) converge to the same value,

```matlab
a = sqrt(mu_s) / (2*tf*ri^(3/2)) * dr; % Equation (6)
```

in the limit of small changes `delta_r = rf - ri`.

Due to the assumptions of these approximations, only asteroids of heliocentric eccentricity less than 0.1 are considered. Fig. 2 shows the thrust accelerations for the 130 asteroids with such an eccentricity. For a fixed `tf` of 10 years, the approximation methods in (3) and (4) yield typical accelerations on the order of 0.1-100 µm/s^2. This range provides us with a baseline for initial guesses for appropriate thrust accelerations. In higher fidelity numerical integrations, described in a later section, we found accelerations in the range 4-20 µm/s^2.


In the comprehensive list of NEAs, there are only 108 asteroids whose size has been calculated. The asteroid radius ranges from 0.03 m to 31.7 m, with an average of 2.67 m. It must be noted that these are only the asteroids whose radius is known with great confidence; there may exist larger asteroids. Using the average radius and assuming a density of `rho_density = 3000`, and a spherical shape, average asteroid mass is approximated to be 239,000 kg Using this average mass and the approximated capture acceleration from (4), the average force requirements are calculated and shown on the right hand side of Fig. 2. The thrust forces and accelerations in Fig. 2 are those required to transport an asteroid from its existing heliocentric orbit into the Earth's heliocentric orbit, without considering phasing.

The asteroid diameters are calculated [35] using absolute magnitude `H` and albedo `A = 0.25;` as

```matlab
A = 0.25;
D = 1329./(10.^(0.2*H).*sqrt(A)); % Equation (7)
```

Although absolute magnitude values are known, the albedo values are approximated as 0.25.

## Computational results

The capture strategy is broken up into two separate stages. Stage 1 is a continuous low-thrust approach to transport the asteroid to a rendezvous with the Earth's sphere of influence. Stage 2 is to create a stable orbit around the Earth after the rendezvous. We choose a constant acceleration thrust that is opposite to the velocity due to its simplicity and its efficiency in changing heliocentric orbital radius. Increasing the magnitude of the acceleration makes it more difficult to physically implement the profile but also decreases the time from the start of the profile to rendezvous with the Earth.

It is important to make sure that the asteroid not only intersects the Earth's orbit, but also has the same heliocentric true anomaly as the Earth when it gets within the sphere of influence of the Earth. We measure the difference in the true anomaly using the angle `phi`, shown in Fig. 4. The phase angle `phi` is the absolute value of the difference in the anomaly of the Earth and the anomaly of the asteroid. The Earth's sphere of influence extends to a radius of approximately 0.01 AU, which means the angular width of the sphere of influence of the Earth can be approximated by a `phi` value of one degree.

The 130 asteroids with `ecc` less than 0.1 are used in the simulation. Of these, due to the two-dimensional assumption of the model, the separation between the orbital planes of the asteroid and Earth, `delta_inc`, needs to be small, where `delta_inc` is defined in (B.4). For each of these asteroids the optimal acceleration is determined by minimizing the value of `phi` (the value for `phi` is only calculated when the asteroid comes within the radius of the Earth). Tao determine the optimal acceleration, the asteroid's equation of motion is solved over the set of accelerations ranging from 2 to 20 µm/s^2. The acceleration resulting in the minimum `phi` (as long as `phi` is less than one degree) is recorded as `LTA_1` and the corresponding flight time `TOF_1`, or 'time of interception' is also recorded; once the stage 2 flight time is determined (see below), the sum is given as the total time-of-flight, `TOF`.


The phase angle is calculated as a function of acceleration for each of the asteroids and the minimum phase angle is determined. Once the asteroid has rendezvoused with the Earth, stage 2 of the capture is initiated. After entering the Earth's sphere of influence, the asteroid's velocity needs to be adjusted in order to create a stable orbit around the Earth. If the asteroid is to continue traveling at its rendezvous velocity, it would most likely not be captured by the Earth. Instead, the asteroid would simply be slingshot back into a different heliocentric orbit.

Two approaches for transforming the asteroid's heliocentric orbit into a captured geocentric orbit (i.e., stage 2) are considered. Firstly, an impulsive force could be applied at a critical moment causing the transformation in the orbit. A second approach is to use low-thrust applied by a continuous acceleration, as in stage 1.

### Capture-impulsive method

The first method is a simple approach that determines an instantaneous change in velocity that would effect a capture to a bound geocentric circular orbit. The necessary `delta_V` is executed at $q$, the close approach distance of the asteroid with the Earth. An example of the capture of asteroid 2008 PG2 using a two-stage method results in an applied stage 1 acceleration of 11 µm/s^2 on 2008 PG2 starting in January 2010, it would rendezvous with the Earth in October 2014, 4.8 years later. The asteroid would be brought within 0.1 degree of the Earth, well within the sphere of influence of the Earth. Subsequently, a `delta_V` of 780 m/s impulse at perigee would circularize the asteroid's hyperbolic trajectory with respect to the Earth, thereby rendering it captured.

As a second example, the capture of 1999 RA32, the 'best' candidate in terms of rapid returna stage 1 acceleration of 20 µm/s^2 would lead to Earth rendezvous in 2.3 years. A subsequent perigee `delta_V` of 2300 m/s would circularize the orbit to roughly half the Moon's semimajor axis. Note that since the incoming trajectory crosses the orbit of the Moon, with proper timing a flyby with the Moon can be used to provide the `delta_V` and thus effect a ballistic capture (without propulsion) [36,37]. In fact, for any asteroid considered, the Stage 1 trajectory can be adjusted to target a desired perigee distance `rp < r_m` such that a Moon flyby can be used to effect a ballistic capture.

### Capture-low-thrust, constant acceleration method

The acceleration requirements for the orbit transformation are derived using the energy balance method. The total Keplerian energy per unit mass (with respect to the Earth) for an asteroid at the point that it enters the Earth's sphere of influence is

```matlab
delta_Energy = m*(0.5*v_inf^2 - mu_E/(rE_SOI)); % Equation (8) where delta_Energy 
```

where `rE_SOI` is the radius of the Earth's sphere of influence, 0.01 AU, `mu_E` is the gravitational constant times the mass of the Earth; and `v_inf` is the excess hyperbolic velocity of the asteroid in the geocentric frame upon entering the sphere of influence (which is far enough from Earth to count as 'infinity'). A positive energy indicates a hyperbolic geocentric trajectory. The energy must be decreased below zero in order to ensure an elliptical orbit around the Earth. The capture acceleration required to remove the excess energy above zero, is shown below in Equation (9).

```matlab
rE_SOI = 0.01; % AU
mu_E = 398600; % km^3/s^2
a_cap = (0.5*v_inf^2 - mu_E/(2*rE_SOI))/(delta_Time*v_inf); % Equation (9)
```

Because the amount of time spent by these asteroids in the Earth's sphere of influence is on the order of weeks, a low-thrust acceleration method may be warranted.

## Appendix A. Estimate of acceleration based on energy balance

The energy required for getting an asteroid from one known orbit to another known orbit can be calculated using conservation of energy. The assumptions for this approximation are as follows:

1. The orbit is approximately a circular orbit about a central body throughout the motion.
2. The radius of the orbit changes at a uniform rate.
3. The force due to thrust is directed opposite to the asteroid's velocity.

The first assumption simplifies the equation of the velocity of the orbit to

```matlab
v = sqrt(mu/r); % Equation (A.1)
```

where `v` is the average velocity of the orbit, `mu=GM`, `G` is the gravitational constant, `M` is the mass of the central body, and `a` is the semi-major axis orbit.

The second assumption allows the time differential to be expressed in terms of the radius differential:

```matlab
dt = tf/(rf-ri)*dr % Equation (A.2)
```

The work done to a system can be evaluated as the integral of the power of the system. Conservation of energy requires this value be equal to the sum of the change in the total energy (kinetic plus potential).

where `Ft` is the thrust force required to move the asteroid, `m` is the mass of the asteroid the result is shown in Equation (A.4).

```matlab
F_T = m*sqrt(mu)/tf*(1/ri-1/rf)*((rf-ri)/(4*(sqrt(rf)-sqrt(ri)))); % Equation (A.4)
```

The required acceleration may be determined by dividing Eq. (A.4) by the mass of the asteroid, `m`, yielding equation (3).

## Appendix B. Estimate of acceleration based on Edelbaum's approximation

For an object in an eccentric orbit the period of the orbit is

```matlab
T = 2*pi*a^(3/2)/sqrt(mu); % Equation (B.2)
```

The difference in inclination angle between an initial and final orbit is given by

$$
\begin{align*}
\Delta i=\cos ^{-1}\left[\frac{\mathbf{L}_{i} \cdot \mathbf{H}_{f}}{\left\|\mathbf{L}_{i}\right\|\left\|\mathbf{L}_{f}\right\|}\right]
\end{align*}\ ~(B.4)
$$
```matlab
Li = dot(ri, vi);
Lf = dot(rf, vf);
delta_inc = acos(dot(Li,Lf)./(norm(Li)*norm(Lf))); % Equation (B.4)
```
where `Li` and `Lf` are the angular momentums per unit mass for the initial and final orbits, respectively.

For two eccentric orbits, we can use the average velocity values for the initial and final orbits, `v` respectively, into Edelbaum's equation (see, e.g, [34]),

$$
\begin{align*}
\Delta V_{e d e l}=\sqrt{v_{i}^{2}+v_{f}^{2}-2 v_{i} v_{f} \cos \left(\frac{\pi}{2} \Delta i\right)}
\end{align*}\ ~(B.5)
$$
```matlab
delta_V_edel = sqrt(vi^2 + vf^2 - 2*vi*vf*cos(delta_inc)); % Equation (B.5)
```

where `delta_V_edel` is the change in velocity required to transform the initial orbit to the final orbit. If this `delta_V` is delivered via a constant acceleration `accel_edel` during a time of flight `tf`, the approximate acceleration is

```matlab
accel_edel = delta_V_edel/tf; % Equation (B.6)
```

We can rewrite this using `ri` and `rf` as the initial and final semimajor axes, respectively, and `ecci` and `eccf` as the initial and final eccentricities, respectively. The result is shown in Equation (B.7).

```matlab
accel_edel = sqrt(mu)/tf*sqrt((1/ri)*(1 - ecci^2/4 + ecci^4)...
           + (1/rf)*(1 - eccf^2/4 + eccf^4)...
           - (2*cos(0.5*pi*delta_inc)/sqrt(rf*ri))...
           * (1 - 1/8*(ecci^2 + eccf^2) + ecci^4 + eccf^4 + ecci^2*eccf^2)) % Equation (B.7)
```

where all the small terms are of order 4 or larger in the eccentricities. If one ignores terms of order 2 or larger in the eccentricities, one obtains (4).
