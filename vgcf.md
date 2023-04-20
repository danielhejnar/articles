# Continuous Low Thrust Trajectory Design and Optimization

## The Virtual Gravity Field Method

### Introduction
Continuous low thrust propulsion is an effective way to achieve space mission trajectory design, which has attracted much attention in the literature. The fundamental task of trajectory design is to find a thrust profile that can change spacecraft from one state to another within a given flight of time. However, the trajectory design is still challenging because too many trajectory parameters need analyzing, and those parameters associated with search space are extensive.

Several authors have proposed analytical solutions. Tsien [21], Boltz [6] and Mengali 16 advanced analytical solutions of orbit motion under continuous thrust aligning along the radius direction. Following the same formulation, Boltz [7] and Zee [25] studied the case of tangential thrust for continuous low-thrust trajectories. Furthermore, Gao [8] presented an averaging technique to obtain an analytical solution for tangential thrust. Although those methods can largely simplify the continuous low thrust problem, they are only suitable for some special cases.

For general cases, there is no analytical solution in continuous thrust trajectory. The low thrust trajectory is designed typically using two optimization methods: the indirect optimization method and the direct optimization method. The former is based on the calculus of variation, and then the optimization problem is modelled as a two-point boundary value problem. However, it is susceptible to the initial guess, so generating suitable solutions using the indirect method is difficult. The latter parameterizes a trajectory using a few variables, and then the nonlinear programming technique is used to optimize those variables to maximize the objective function. A variety of methods of this type have been examined [5]. A noticeable direct method was proposed by Jon A. Sims et al. [11]. In his work; the trajectory was divided into several segments at discrete points. The continuous low thrust trajectory design problem was modelled into a nonlinear programming problem, and was solved by the nonlinear programming software `SNOPT`. Another method that can provide an initial guess for more accurate optimizers is the shape-based method developed by [2]. In the shape-based method, the thrust is assumed to be aligned along the velocity direction, and the radical vector of the spacecraft is written as a function of the transfer angle. Then the coefficients of the function are calculated to satisfy the boundary constraints. Later, 1 extended the shape-based approximation method to reduce the required thrust to satisfy the thrust constraint. In their method, the radius vector and polar angle are described as functions of flight time in the form of Fourier series. Those Fourier series coefficients are optimized using Fmincon tools in Matlab software to satisfy the boundary and thrust constraints.

This section proposes a virtual central gravitational field method (VCGF) to determine continuous thrust trajectory. Instead of providing some particular initial guesses, the method can efficiently generate many feasible initial guesses and find the optimal one. There is no prior assumption about the direction of thrust used in this method, and the virtual gravity analytically determines the solutions. The basic idea of the method is that, without thrust and neglecting perturbation, the spacecraft flies in a conic orbit in a two-body gravitational field. Similarly, suppose the thrust and the Earth's gravity can form a virtual central gravitational field. In that case, the spacecraft can fly in a virtual conic orbit to accomplish trajectory maneuvers in that virtual gravity. In this way, feasible continuous thrust trajectories can be parameterized and expressed as a kind of displaced orbit named virtual conic orbits analytically. Combined with the Particle Swarm Optimization (PSO) algorithm, the proposed method provides a new way to obtain an initial guess given an objective function. This method is intended not only for rendezvous cases but also for solving less constrained cases like orbit interception.

### The Definition of Virtual Central Gravitational Field

In the two-body problem, spacecraft flies in a Keplerian orbit in the geocentric gravitational field with no thrust. By analogy, if the continuous thrust and the Earth's gravity can form a virtual central gravitational field, the spacecraft can fly in a virtual Keplerian orbit. A virtual central gravitational field can be defined by two parameters: the magnitude of virtual gravity $\mu_{v g}$, the displaced position parameter $r_{0}$, as shown in Fig. 1. From the definition of the virtual central gravitational field, we can see that the Earth's gravity is a special case of a virtual gravity, whose gravity parameter is $\mu_{v g}=1 \mathrm{DU/TU}^{2}$, $\boldsymbol{r}_{0}=0$, where $\mathrm{DU}$ is the distance unit defined as $1 ~\mathrm{DU} = 1 ~(\mathrm{AU}) = 149,596,600 ~\mathrm{km}$ and $\mathrm{TU}$ is the time Unit defined as $1 ~\mathrm{TU} = \frac{1}{2}\pi ~\mathrm{year} = 58.17 ~\mathrm{days} = 5,025,888 ~\mathrm{seconds}$.

As shown in Fig. 2, point $\mathrm{A}$ is an initial point and point $\mathrm{B}$ is a final point; $r_{a}, r_{b}$ are position vectors in point A and point B in geocentric coordinate system; $r_{v g a}, r_{v g b}$ are position vectors in point $\mathrm{A}$ and point $\mathrm{B}$ in the virtual central gravitational coordinates system. Virtual Keplerian orbits can be classified into three types as shown in Fig. 

(1) $\mu \neq \mu_{v g}, r_{0}=0$ as shown in Fig. 2a;

(2) $\mu \neq \mu_{v g}, r_{0} \neq 0$ as shown in Fig. 2b; 

![](https://cdn.mathpix.com/cropped/2023_04_19_7439cb6d989663a1bd4eg-03.jpg?height=452&width=530&top_left_y=183&top_left_x=485)

Fig. 1 The virtual central gravitational field

![Fig. 2(a)](https://cdn.mathpix.com/cropped/2023_04_19_7439cb6d989663a1bd4eg-03.jpg?height=397&width=490&top_left_y=773&top_left_x=252)
Fig 2(a) $\mu \neq \mu_{v g}, r=0$

![Fig. 2(b)](https://cdn.mathpix.com/cropped/2023_04_19_7439cb6d989663a1bd4eg-03.jpg?height=447&width=511&top_left_y=725&top_left_x=754)
Fig 2(b) $\mu \neq \mu_{v g}, r \neq 0$

The VCGF method requires that a spacecraft fly in a virtual conic orbit and the virtual central gravitational field to satisfy the boundary constraints. There are a few steps in the VCGF method. Firstly, a feasible virtual gravity field determined by the parameter set $\left(\mu_{v g}, r_{0}\right)$ is required, in which the spacecraft runs in the virtual conic orbits to satisfy trajectory constraints. Secondly, the thrust necessary to form the virtual gravity is computed, and the fuel consumption is calculated. Finally, considering the objective function, the PSO algorithm (`particleswarm`) MATLAB function `particleswarm` is used to find the optimal initial guess for a more accurate optimizer.

## Trajectory Design Using the VCGF Method

### Rendezvous Trajectory Design Using VCGF Method

### Trajectory Parameterization

In the geocentric coordinate system, $r_{a}$ and $v_{a}$ are the position vector and the velocity vector at point $\mathrm{A}, r_{b}$ and $v_{b}$ are the position vector and the velocity vector at terminal point B. $\left(\mu_{v g}, r_{0}\right)$ are parameters of the virtual gravity. In the virtual central gravitational coordinate system, $r_{v g a}$ and $v_{v g a}$ are the position vector and the velocity vector at point A respectively; $r_{v g b}$ and $v_{v g b}$ are the position vector and the velocity vector at point $B$ respectively. Velocity vectors and position vectors of the spacecraft at initial point A and terminal point B can be computed as,

$$
\begin{align}
\left\{\begin{array}{l}
\boldsymbol{r}_{v g a}=\boldsymbol{r}_{a}+\boldsymbol{r}_{0} \\
\boldsymbol{v}_{v g a}=\boldsymbol{v}_{a} \\
\boldsymbol{r}_{v g b}=\boldsymbol{r}_{b}+\boldsymbol{r}_{0} \\
\boldsymbol{v}_{v g b}=\boldsymbol{v}_{b}
\end{array}\right.
\end{align}
$$

With the Keplerian orbit theory, one has that,

$$\begin{align}
\left\{\begin{array}{l}
\{boldsymbol{h}_{v g}=\boldsymbol{r}_{v g a} \times \boldsymbol{v}_{v g a}=\boldsymbol{r}_{v g b} \times \boldsymbol{v}_{v g b} \\
\Delta f=\arccos \left(\frac{\boldsymbol{r}_{v g a} \cdot \boldsymbol{r}_{v g b}}{\boldsymbol{r}_{v g} \cdot \boldsymbol{r}_{v g b}}\right) \\
r_{v g a}=\frac{h_{v g}{ }^2}{\mu_{v g}} \frac{1}{1+e_{v g} \cos f_{v g a}} \\
f_{v g b}=f_{v g a}+\Delta f \\
r_{v g b}=\frac{h_{v g}{ }^2}{\mu_{v g}} \frac{1}{1+e_{v g} \cos f_{v g b}}
\end{array}\right.
\end{align}$$

where $\boldsymbol{h}_{v g}, e_{v g}$ are the angular of momentum and the eccentricity of the virtual Keplerian orbit, while $f_{v g a}, f_{v g b}$ are the true anomaly of point A and B in the virtual central gravitational field respectively. The range of parameter $f_{v g a}$ is $[0,2 \pi]$, while $r_{0}$ should satisfied,

$$
\begin{align}
\boldsymbol{h}_{v g}=\boldsymbol{r}_{v g a} \times \boldsymbol{v}_{v g a}=\boldsymbol{r}_{v g b} \times \boldsymbol{v}_{v g b}
\end{align}
$$

In the X-Y plane, we set

$$
\begin{align*}
\boldsymbol{r}_{a} & =\left[r_{a x}, r_{a y}\right], \boldsymbol{r}_{b}=\left[r_{b x}, r_{b y}\right] \\
\boldsymbol{v}_{a} & =\left[v_{a x}, v_{a y}\right], \boldsymbol{v}_{b}=\left[v_{b x}, v_{b y}\right]
\end{align*}
$$

and then Eq. 3 can be expressed as follow,

$$
\begin{align}
a_{1} \cdot r_{0 x}+b_{1} r_{0 y}=c_{1}
\end{align}
$$

Where $a_{1}, b_{1}, c_{1}$ are expressed as,

$$
\begin{align}
\left\{\begin{array}{l}
a_{1}=v_{a y}-v_{b y} \\
b_{1}=v_{b x}-v_{a x} \\
c_{1}=\left(r_{b x} v_{b y}-r_{b y} v_{b x}\right)-\left(r_{a x} v_{a y}-r_{a y} v_{a x}\right)
\end{array}\right.
\end{align}
$$

![Fig. 3](https://cdn.mathpix.com/cropped/2023_04_19_7439cb6d989663a1bd4eg-05.jpg?height=388&width=513&top_left_y=197&top_left_x=828)
Fig. 3 Orbit rendezvous in 2-D space

Fig. 4 Orbit rendezvous in 3-D space
![Fig. 4](https://cdn.mathpix.com/cropped/2023_04_19_7439cb6d989663a1bd4eg-05.jpg?height=472&width=513&top_left_y=618&top_left_x=828)

In the 2-dimensional plane, through Eq. 4, it can be obtained that vector $\boldsymbol{r}_{0}=$ $\left[r_{0 x}, r_{0 y}\right]$ is located on a line, because the intersection angle between $v_{v g a}$ and $v_{v g b}$ is smaller than $\pi$, the range of $r_{0}$ can be determined as $r_{0 x} \in\left(x_{r l}, x_{r u}\right)$, as shown in Fig. 3 .

Once parameters $\left(r_{0 x}, f_{v g a}\right)$ are given, $\left(\boldsymbol{r}_{0}, \mu_{v g}\right)$ can be calculated using Eqs. 2 and 4. Each parameter set $\left(\boldsymbol{r}_{0}, \mu_{v g}\right)$ can determine a virtual gravity. Therefore, all those feasible trajectories can be parameterized as virtual conic orbits (Fig. 3), and analytically expressed as virtual conic orbits. The process of finding rendezvous trajectory for 3-D space is similar as that for 2-D space, as shown in Fig. 4.

In the VCGF method, the transfer angle for one single virtual conic orbit is less than $2 \pi$, because the spacecraft returns to the initial state after one revolution in the virtual gravity. For multi-revolutions trajectory design problem, the orbit patching technique discussed in [20] is used. The whole trajectory is divided into a few segments at discrete points, and those discrete points are target points or just the control points. Each segment is a virtual conic orbit, and the continuous low thrust trajectory design can be transformed into multi-segments patching problem. More detail can be obtained in Ref. [20].

### Trajectory Optimization

Through the Keplerian orbit theory, the flight time of a spacecraft can be expressed as follows, 

$$
f_{t}=f\left(f_{v g a}, \boldsymbol{r}_{0 x}\right)=\int_{f_{v g a}}^{f_{v g b}} \frac{\sqrt{a_{v g}{ }^{3}\left(1-e_{v g}\right)^{3}}}{\mu_{v g}} \frac{1}{\left(1+e_{v g} \cos f\right)^{2}} d f
$$

Similarly, the energy consumption can be expressed as,

$$
f_{e}=f\left(f_{v g a}, \boldsymbol{r}_{0 x}\right)=\int_{f_{v g a}}^{f_{v g b}}\left(\boldsymbol{F}_{2}-\boldsymbol{F}_{1}\right) d f
$$

here set $r$ is the position vector from the spacecraft to the Earth,

$$
\boldsymbol{F}_{1}=\frac{\mu}{r^{2}}, \boldsymbol{F}_{2}=\frac{\mu_{v g}}{r_{v g}{ }^{2}}, r_{v g}=\frac{h_{v g}{ }^{2}}{\mu_{v g}} \frac{1}{1+e_{v g} \cos f_{v g}}
$$

In minimum fuel consumption case, the objective function is,

$$
f_{\Delta V}=f_{\min }\left(f_{v g a}, \boldsymbol{r}_{0 x}\right)=\int_{f_{v g a}}^{f_{v g b}}\left(\frac{\mu_{v g}}{r_{v g}{ }^{2}}-\frac{\mu}{r^{2}}\right) d f
$$

In this optimization problem, $\boldsymbol{r}_{0 x}$ and $\mu_{v g}$ are independent variables. Feasible variables $\boldsymbol{r}_{0 x}, \mu_{v g}$ need to be optimized, so as to minimize the flight time or the fuel consumption.

### Thrust Acceleration

In the geocentric coordinate system, equation of motion of the spacecraft under continuous thrust can be written as,

$$
\frac{d^{2} \boldsymbol{r}}{d t^{2}}+\frac{\mu}{r^{3}} \boldsymbol{r}=\boldsymbol{T}_{a c}
$$

Here we assume that the thrust can form a virtual gravity, and then in the virtual central gravitational coordinate system, the equation of spacecraft motion under continuous thrust can be written as,

$$
\frac{d^{2} \boldsymbol{r}_{v g}}{d t^{2}}+\frac{\mu_{v g}}{r_{v g} 3} \boldsymbol{r}_{v g}=0
$$

where

$$
\boldsymbol{r}_{v g}=\boldsymbol{r}-\boldsymbol{r}_{0} .
$$

Because $\boldsymbol{r}_{0}$ is a constant in one virtual gravity, we have $\left(\boldsymbol{r}_{0}\right)^{\prime}=0$. The position vector and the velocity vector in the virtual centric coordinate system can be computed by Eq. 13 Fig. 5 Force analysis in the 2-dimensional space

![](https://cdn.mathpix.com/cropped/2023_04_19_7439cb6d989663a1bd4eg-07.jpg?height=414&width=513&top_left_y=195&top_left_x=828)

Fig. 6 Force analysis in the 3-dimensional space

![](https://cdn.mathpix.com/cropped/2023_04_19_7439cb6d989663a1bd4eg-07.jpg?height=448&width=510&top_left_y=641&top_left_x=829)

$$
\left\{\begin{array}{l}
\boldsymbol{r}_{v g}=\boldsymbol{r}-\boldsymbol{r}_{0} \\
\boldsymbol{v}_{v g}=\boldsymbol{v}
\end{array}\right.
$$

where $\boldsymbol{v}, \boldsymbol{v}_{v g}$ are velocity vectors in geocentric coordinates system and virtual central gravitational coordinate system. Through Eqs. 10, 11 and 12, the required thrust acceleration (TA) is,

$$
\boldsymbol{T}_{a c}=\frac{d^{2} \boldsymbol{r}}{d t^{2}}+\frac{\mu}{\left|\boldsymbol{r}^{3}\right|} \boldsymbol{r}-\frac{d^{2} \boldsymbol{r}_{v g}}{d t^{2}}-\frac{\mu_{v g}}{\left|\boldsymbol{r}_{v g}^{3}\right|} \boldsymbol{r}_{v g}
$$

As shown in Fig. 5, in 2-dimensional space, given $\boldsymbol{r}_{0} x, \mu_{v g}$, the thrust acceleration required in 2-D space can be obtained by Eq. 15

$$
\left\{\begin{array}{l}
\boldsymbol{r}_{v g}=\boldsymbol{r}-\boldsymbol{r}_{0} \\
T_{a r 1}=\frac{\mu}{|\boldsymbol{r}|^{2}} \\
T_{a r 2}=\frac{\mu_{v g}}{\left|\boldsymbol{r}_{v g}\right|^{2}} \\
T_{a c}=T_{a r} \cdot \sin \left(\arccos \left(\frac{T_{a r 2}}{T_{a r 1}}\right)\right)
\end{array}\right.
$$

where $T_{a r 1}, T_{a r 2}, T_{a c}$ are magnitude of gravity acceleration, virtual central gravitational acceleration and required TA respectively.

In case of 3-dimensional trajectory design, as shown in Fig. 6, $r_{M}$ is the position vector of spacecraft at point $\mathrm{M}$, and $\boldsymbol{r}_{v g M}$ is the position vector at point $\mathrm{M}$ in the virtual gravity. Assuming TA is $\boldsymbol{T}_{a c}=\left[T_{a c n 2}, T_{a c \theta 2}\right]$, where $T_{a c n 2}$ is aligned along the radical direction, and $T_{a c \theta 2}$ is aligned along the circulation direction; $T_{a r 1}$ is the geocentric gravitation force acceleration, and $T_{a r 2}$ is the virtual centric gravitational force acceleration; $\alpha$ is the angle between $T_{a c \theta 2}$ and $T_{a r 2}$, and $\beta$ is the angle between $\boldsymbol{r}_{M 1}$ and $\boldsymbol{h}_{2}$, then the required TA can be computed as,

$$
\left\{\begin{array}{l}
\beta=\arccos \frac{\left|\boldsymbol{r}_{M} \cdot \boldsymbol{h}_{v g}\right|}{\left|\boldsymbol{r}_{M}\right| \cdot\left|\boldsymbol{h}_{v g}\right|} \\
T_{a r 1}=\frac{\mu}{r_{M}^{2}} \\
T_{a r 2}=\frac{\mu_{v g}}{r_{v g} M^{2}} \\
\alpha=\arcsin \left(\frac{T_{a r 2}}{T_{a r 1} \sin \beta}\right) \\
T_{a c \theta 2}=T_{a r 2} \sin \beta \cos \alpha \\
T_{a c n 2}=T_{a r 1} \cos \beta
\end{array}\right.
$$

### Orbit Interception Trajectory Design Using VCGF Method

The VCGF method can also be applied to designing less constrained interception trajectory. In Eq. 2, the unknown variables $\boldsymbol{r}_{0}, f_{v g a}$ or $\boldsymbol{r}_{0}, \mu_{v g}$ and the virtual conic orbit in Fig. 2a are used. In two dimension problem, we can set $\boldsymbol{r}_{0}=0$ to simplify the problem, as shown in Fig. 7. Given parameters $\boldsymbol{r}_{a}, \boldsymbol{r}_{b}, \boldsymbol{v}_{a}$, it needs to obtain feasible parameter $\mu_{v g}$ to solve Eq.  While in the three dimension problem, both $\boldsymbol{r}_{0}, \mu_{v g}$ are required to optimize. The root finding function `fzero` in Matlab is be used to solve this problem.

In the PSO algorithm (`particleswarm`), the free parameters are the magnitude of virtual gravity $\mu_{v g}$ and the projection of $\boldsymbol{r}_{0}$ in $x$ axis: $\boldsymbol{r}_{0}$. The objective function can be the fuel cost of whole trajectory or the flight time. Here we set the maximal number of iteration in PSO algorithm (`particleswarm`) are $\mathrm{N}$. Take the orbit rendezvous trajectory design as an example. Given the range of free variables $r_{0 x} \in\left(r_{0 x l}, r_{0 x u}\right)$, the initial point $\left(\boldsymbol{r}_{A}, \boldsymbol{v}_{A}\right)$ and the terminal point $\mathrm{B}\left(\boldsymbol{r}_{B}, \boldsymbol{v}_{B}\right)$.

There is a large amount of parameter set $\left(\boldsymbol{r}_{0}, \mu_{v g}\right)$, and each one corresponds to a trajectory, and the feasible trajectories are those can satisfy boundary constraints.

![](https://cdn.mathpix.com/cropped/2023_04_19_7439cb6d989663a1bd4eg-08.jpg?height=339&width=513&top_left_y=1714&top_left_x=828)
Fig. 7 Optimal trajectory using PSO algorithm (`particleswarm`)

The PSO algorithm (`particleswarm`) is adopted to obtain feasible parameter set $\left(\boldsymbol{r}_{0}, \mu_{v g}\right)$ from their ranges to determine the virtual gravity and the corresponding virtual conic orbits. There are a few steps in the algorithm. To begin with, the range of free parameters is set, and a large number of possible trajectories, same as the number of parameter sets $\left(\boldsymbol{r}_{0}, \mu_{v g}\right)$, are obtained. Furthermore, those feasible trajectories which can satisfy the constraints are chosen. Finally, the value of the objective function is calculated, and the optimal solution is obtained. The process of PSO algorithm (`particleswarm`) is listed as follow:

Step1: Find the ranges of free variables $\mu_{v g} \in\left(\mu_{v g l}, \mu_{v g u}\right), r_{0 x} \in\left(r_{0 x l}, r_{o x u}\right)$;

Step2: Initialize the particle position with a uniformly distributed random vector $\left(r_{0 x}, \mu_{v g}\right)$ in their ranges;

Step3: Calculate $r_{0 y}$ through Eq. 4; then the parameter set $\left(\boldsymbol{r}_{0}, \mu_{v g}\right)$ is obtained;

Step4: Calculate the radius and velocity state of initial point $\mathrm{A}\left(\boldsymbol{r}_{v g a}, \boldsymbol{v}_{v g a}\right)$, and terminal point $\mathrm{B}\left(\boldsymbol{r}_{v g b}, \boldsymbol{v}_{v g b}\right)$ in the virtual gravity through Eq. 1;

Step5: Calculate transfer angle from point $\mathrm{A}$ and point $B$ in virtual gravity field through Eq. 2

Step6: Calculate the orbital elements of point $A$ through its radius and velocity in virtual gravity field, $\operatorname{coe}_{v g A}=\left[a_{v g A}, e_{v g A}, i_{v g A}, w_{v g A}, \Omega_{v g A}, T A_{v g A}\right]$, here $T A_{v g A}$ are the virtual conic orbital inclination, argument of the periapsis, longitude of the ascending node, and true anomaly respectively.

Step7: Calculate the true anomaly at point $B$ in the designed virtual gravity $\left(T A_{v g b}=T A_{v g a}+\Delta \theta\right)$

Step8: Calculate the radius and velocity $\boldsymbol{r}_{v g b}, \boldsymbol{v}_{v g b}$ through the orbital elements, $\operatorname{coe}_{v g B}=\left[a_{v g A}, e_{v g A}, i_{v g A}, w_{v g A}, \Omega_{v g A}, T A_{v g B}\right]$

Step9: Calculate the radius error $\Delta \varepsilon_{r}=\left|\boldsymbol{r}_{v g B}-\boldsymbol{r}_{v g b}\right|$ and the velocity error $\Delta \varepsilon_{v}=\left|\boldsymbol{v}_{v g B}-\boldsymbol{v}_{v g b}\right| ;$ if $\Delta \varepsilon_{r} \leq 10^{-4}, \Delta \varepsilon_{V} \leq 10^{-4}$; the trajectory in the virtual gravity can satisfy the boundary constraint. Then record the parameter set $\left(\boldsymbol{r}_{0}, f_{v g a}\right)_{i}$, and calculate the fuel consumption (or the flight time) as its fitness; else if $\Delta \varepsilon_{r}>$ $10^{-4}, \Delta \varepsilon_{V}>10^{-4}$; then set the fitness of this particle as 1 ;

Step10: Initialize the particle's best-known position to its initial position; then update the swarms best known position and its fitness; If the number of iteration $i<N$; return to step4. If the number of iteration $i>N$, finished; It should be noted that, there is a large number of feasible virtual Keplerian orbits that can satisfy boundary constraints, and the optimal one is the subcategory of them.

### Mission Applications
In order to verify the effectiveness of the proposed method, three application examples of the VCGF method are presented in this section. The first one is the Earth-Mars orbital transfer trajectory design. The proposed method was used to generate initial guesses of continuous thrust rendezvous trajectory. Those solutions were compared with the shape-based (SB) method given in [24], in terms of the magnitude and the direction of thrust, transfer angle and fuel consumption. The second example is a fuel-optimal Earth-Mars-Ceres flight trajectory mission discussed in the same reference. The whole trajectory consists of an interception trajectory from the Earth to the Mars, and a rendezvous trajectory from the Mars to the Ceres. The effectiveness of the VCGF method is evaluated by providing an initial guess for the direct optimizer. Solutions of the VCGF method are compared with the three-dimension, shape-based method solution discussed in [24]. Here the direct optimizer GPOPS, a Matlab software for solving a nonlinear optimal control problems, is selected to generate an accurate solution based on the proposed method and the shape-based method. The last case is a collision-speed-maximal interception trajectory design problem in Ref. [12]. In this example, a spacecraft is transferred from the Earth to intercept with a hazardous asteroid. The asteroid 99942 Apophis is the potentially hazardous asteroid on an impact trajectory toward the Earth. The aim of optimization is to maximize the relative speed between the spacecraft and the asteroid. For well-documented reasons, heliocentric canonical units were used. In this paper, the distance units $(\mathrm{AU})$ and the time units $(\mathrm{TU})$ are: $1 \mathrm{DU}=.1(\mathrm{AU})=149596000 \mathrm{~km}, 1$ $\mathrm{TU}=1 / 2 \pi$ year $=58.17$ days All of the examples have been performed on an Intel Core $6 \mathrm{GHz}$ with Windows 8 . The computation time of optimization is calculated by MATLAB tic-toc command.

Example A: The Earth-Mars orbit rendezvous

In this example, the two-dimension Earth-Mars orbit is performed. A spacecraft is transferred from the Earth to rendezvous with the Mars. The boundary conditions are listed in Table 1. Here, in order to analyze the relationship between the transfer angle, the required thrust and the fuel cost, the VCGF method is applied to generate initial guesses in case of three transfer angles $(\Delta \theta)$ : a half revolution $(\Delta \theta=\pi)$, one revolution ( $\Delta \theta=2 \pi$ and two revolutions $(\Delta \theta=4 \pi)$. Using the VCGF method, those feasible trajectories are expressed as virtual conic orbits and parameterized by the parameter set $\left(\boldsymbol{r}_{0}, \mu_{2}\right)$. For one virtual conic trajectory, the transfer angle is less than $2 \pi(\Delta \theta<2 \pi)$. While in case of multi-revolutions condition $(\Delta \theta>2 \pi)$, the whole trajectory is divided into a few segments at discrete points, and each segment corresponds to a virtual conic orbit. In order to testify the suitability of proposed method, the results are compared with the five-degree inverse polynomial shaped based method, in terms of magnitude and direction of required thrust, the flight time and the fuel consumption. The boundary conditions of the Earth-Mars rendezvous mission are listed in Table 1.

The transfer angles of those cases are assumed to be $\pi, 2 \pi, 4 \pi$, respectively. The corresponding flight times are set as 3.75TU, 7.683TU and 16.69 TU respectively. Trajectories generated by the VCGF method and the shape-based method are shown in Fig. 8 (for the case of $n=2$ ). The required thrust for all three cases is shown in Figs. 9, 10 and 11. Table 2 shows the parameters of the virtual gravity

Table 1 Initial and final conditions for transfer trajectory
|$r_{i}=1 A U$    | $\theta_{i}=0$      | $\dot{r}_{i}=0$ | $\dot{\theta}_{i}=0.6564$ |
|------------------------|--------------------|-----------------|---------------------------|
|$r_{f}=1.52 A U$ | $\theta_{f}=n \pi$ | $\dot{r}_{f}=0$ | $\dot{\theta}_{f}=0.5333$ |
