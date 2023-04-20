# Implementation of a Low-Thrust Trajectory Optimization Algorithm for Preliminary Design

## Abstract
A tool developed for the preliminary design of low-thrust trajectories is described. The trajectory is discretized into segments and a nonlinear programming method is used for optimization. The tool is easy to use, has robust convergence, and can handle many intermediate encounters. In addition, the tool has a wide variety of features, including several options for objective function and different low-thrust propulsion models (e.g., solar electric propulsion, nuclear electric propulsion, and solar sail). High-thrust, impulsive trajectories can also be optimized.

## Introduction
Electric propulsion systems, while highly efficient, produce only a small amount of thrust. As a result, the engines operate during a significant fraction of the trajectory. This characteristic makes it much more difficult to find optimal trajectories. The methods for optimizing low-thrust trajectories are typically categorized as either indirect or direct. Indirect methods are based on calculus of variations, resulting in a two-point boundary value problem that is solved by satisfying terminal constraints and targeting conditions. These methods are subject to extreme sensitivity to the initial guess of the variables some of which are not physically intuitive. Adding a gravity assist to the trajectory compounds the sensitivity. Direct methods parameterize the problem and use nonlinear programming techniques to optimize an objective function by adjusting a set of variables. A variety of methods of this type have been examined with varying results. These methods are subject to the limitations of the nonlinear programming techniques.

In this program we describe the implementation of a direct method intended to be used primarily for preliminary design of low-thrust interplanetary trajectories, including those with multiple gravity assists. Preliminary design implies a willingness to accept limited accuracy to achieve an efficient algorithm that executes quickly. The basic approach and testing of a prototype was described by Sims and Flanagan. The underlying structure in the current implementation remains the same, although a few of the optimization variables have changed slightly. Many new features and functionalities have been added, and a graphical user interface has been developed to assist users in creating starting guesses and visualizing and analyzing results.

## Approach

### A. Trajectory Structure

The trajectory is divided into legs that begin and end at control nodes (Fig. 1). Typically, the control nodes are associated with planets or small bodies, but they can be free points in space (i.e., non-body control nodes). On each leg is a single match point, and the trajectory is propagated forward in time from the leg's earlier control node to the match point and backward from the leg's later control node to the match point. This technique significantly reduces the sensitivity of the propagation to intermediate flybys compared to propagating strictly forward in time from the beginning of the trajectory to the end. Several optimization variables are available at each control node, providing considerable and direct control over the trajectory when needed.

Continuous thrusting is modeled as a series of impulses. The legs are subdivided into segments with an impulsive `delta_V` (change in velocity) in the middle of each segment. When modeling low-thrust propulsion systems, the magnitude of the impulse is limited by the amount of `delta_V` that could be accumulated over the duration of the segment.

The propagation between control nodes, impulses, and match points is according to a two-body model with respect to a primary body. Flybys of secondary bodies are modeled as instantaneous changes in the direction of the `V_inf` (relative velocity vector). For interplanetary trajectories, the Sun is the primary body and planets (or small bodies such as asteroids and comets) are the secondary bodies. We can also model planetocentric trajectories with a planet as the primary body and the moons of the planet as secondary bodies.

![Figure 1 Trajectory Structure](https://cdn.mathpix.com/cropped/2023_04_20_2fd238caec09d9e31074g-02.jpg?height=1166&width=1185&top_left_y=1184&top_left_x=470)
_Figure 1. Trajectory Structure. A feasible trajectory is continuous in position, velocity, and mass at the match point on each leg. The trajectory depicted in Fig. 1 is in an intermediate, unconverged state with noticeable position discontinuities at the match points._

An encounter with a body represented by a control point can be a flyby or a rendezvous. For a rendezvous, the trajectory will match both the position and velocity of the body at the time of arrival. The mass of the arrival body is not modeled in this rendezvous case. For a flyby, the trajectory will match the position of the body at the time of arrival but not necessarily the velocity. For a flyby of an intermediate body, the effect of the mass of the body is modeled by a rotation of the `V_inf`, and the flyby altitude is determined based on the mass.

### B. Optimization

The trajectory structure described above leads to a constrained, nonlinear optimization problem which we solve using the nonlinear programming library SNOPT in MATLAB. 

#### Variables

The independent variables at each control node associated with a body, such as a planet, are the velocity of the spacecraft relative to the body, the mass of the spacecraft, and the corresponding epoch. At an intermediate control node, the variables are, in general, different upon arrival at the control node compared to departure from the control node. For example, during a flyby, the relative velocity vector changes. For an intermediate body rendezvous, the departure time may be different than the arrival time. The mass of the spacecraft can also change due to a mass drop such as a probe release. For these reasons, each intermediate control node has two sets of variables: one at arrival and one at departure. For the initial control node, only the departure variables apply, and for the final control node, only the arrival variables apply. The independent variables at a non-body control node are the spacecraft mass, position, and velocity relative to the primary, and the corresponding epoch. Again, intermediate non-body control nodes have two sets of variables.

The other major set of independent variables is composed of the components of the thrust vector on each segment. These thrust vector variables represent the vast majority of independent variables. By discretizing the thrust profile as a function of time, we have converted the infinite-dimensional continuously varying thrust problem into a finite-dimensional problem, but the number of these variables still drives the overall scale of the problem and makes it numerically challenging.

Other potential independent variables are the reference power of the spacecraft and the specific impulse of the thrusters in the case that the specific impulse is constant throughout the trajectory.

#### Bounds and Constraints

Upper and lower bounds are placed on all of the independent variables. In addition, constraints can be placed on functions of these independent variables. The value of each constraint function must lie between an upper and lower limit. A primary constraint on the optimization is that the position, velocity, and mass of the spacecraft must be continuous at the match points, i.e., the state from the forward propagation to a match point must be the same as that from the backward propagation to that point. The magnitude of the thrust on each segment is constrained due to the limited power available for thrusting. For the initial control node, the mass can be constrained to lie on or below a specified launch vehicle performance curve, and the magnitude and declination of the departure `V_inf` can be constrained. At any rendezvous body, the magnitude of the `V_inf` is constrained to be zero. At an intermediate body flyby, the magnitude of the arrival `V_inf` and the flyby radius can be constrained, and the magnitude of the departure `V_inf` is constrained to be equal to that of the arrival `V_inf`.

Constraints can be placed on flight time and propellant mass consumed between any two control nodes, and the minimum distance from the spacecraft to the Sun can also be constrained.

#### Cost functions

The following optimizations can be performed:

1. Maximize the final net spacecraft mass (final spacecraft mass - propulsion system mass)

2. Minimize the initial spacecraft mass (subject to a lower bound on the final spacecraft mass)

3. Minimize the total trip time

4. Optimize a weighted combination of the final net spacecraft mass and the total trip time

5. Maximize a function corresponding to the change in the Earth miss distance after impacting a small body. This is the cost function that was used in the first Global Trajectory Optimisation Competition sponsored by the Advanced Concepts Team of the European Space Agency.

## Optimization

The optimizer adjusts the independent variables to satisfy all of the bounds and constraints while simultaneously optimizing the cost function. If the optimizer can find a set of variables that satisfies the bounds and constraints, the trajectory defined by those variables is said to be feasible. If, in addition, the optimizer succeeds in locally optimizing the selected cost function, then the trajectory is said to be optimal and the optimization problem has been solved. There may be many locally optimal solutions within the space defined by the set of variables and the corresponding bounds and constraints. The solution actually found depends to a large extent on the initial guess provided for the variables.

During the optimization process, the program computes not only the values of all constraint functions and the cost function, but it also computes the partial derivatives of all constraint functions and the cost function with respect to each variable. The derivatives are formulated analytically (not by finite differencing), and their computation represents the bulk of the effort and code. The optimizer uses these derivatives to refine iteratively the values of the variables. We have found analytic derivatives to be a key component leading to fast execution and robust convergence.

## Program Components

For a given set of variables provided by the optimizer, the Core computes the values of the constraint and objective functions and the partial derivatives of all constraints and the cost function with respect to each variable, and returns these to the optimizer. These computations require the propagation of the trajectory as described above.

### Overview

The variables defining a trajectory (described in Section II B) are represented by the independent variables vector `x` `x`. Given an initial guess for `x`, it is the Core's task, combined with the optimizer SNOPT, to find a value of `x` that represents a feasible and locally optimal solution to the trajectory optimization problem.

The independent variables contained in `x` are constrained to be within upper and lower bounds: `x_lb <= x <= x_ub % equation (1)`.

where `x_lb` and `x_ub` are vectors of lower and upper bounds for the components of `x` and must be provided as inputs to the Core. For a given `x`, Core routines compute the trajectory constraint function vector `F(x)`. The components of `F` represent trajectory values that must be constrained for the trajectory to be feasible. For example, contained within `F` (i.e., as components of `F` ) are the position, velocity and mass mismatches at the trajectory match points (these are constrained to be zero to enforce position, velocity and mass continuity along the trajectory). Other components of `F` represent the vector magnitude of the `delta_Vs` for each thrust segment (these are constrained to be less than or equal to the maximum `delta_V` attainable by the low thrust acceleration over the segment duration). These are only two of many constraint functions contained within `F`. The lower and upper limits that constrain the values of `F(x)` are inputs to the Core and represented by `F_ll` and `F_ul`: `F_ll <= F(x) <= F_ul % equation (2)`

If Eq. 1 and Eq. 2 are satisfied, the trajectory is said to be feasible. Beyond feasibility, the other goal of the optimizer is to make the trajectory optimal, which is done by maximizing some property of the trajectory. For example, the goal may be to maximize the final spacecraft mass, or to minimize the flight time (this is achieved by maximizing the negative of the flight time). The property to be maximized is computed by the cost function `C(x)`, which is a scalar and is also a function of the independent variables `x`.

`C(x) = maximum % equation (3)`

It is the job of the optimizer to maximize the cost function `C(x)` while simultaneously satisfying the constraints of the problem (Eqs. 1 and 2). To achieve this, the optimizer iteratively adjusts the components of `x` taking steps toward an optimal solution. For each step, new values for `F(x)` and `C(x)`are computed. In addition to computing the values `F(x)` and `C(x)`, the Core also computes the partial derivatives `dF/dx` (the Jacobian of `F`) and `dC/dx` (the gradient of C) which are required by the optimizer to determine the direction and magnitude for the next step. These derivatives are computed analytically (rather than using a finite differencing scheme). Robust convergence depends upon the quality of these derivatives and much of the computational effort of the Core is dedicated to their accurate and efficient computation. The Core (and the optimizer) also exploit the fact that the Jacobian `dF/dx` is often very sparse (i.e., most of the elements of `dF/dx` are zero). Exploiting the sparsity of the Jacobian greatly enhances optimization performance.

If the constraints are satisfied (Eq. 1 and Eq. 2) and the cost is locally maximized (Eq. 3), then the optimization problem is solved and a local solution has been found. There may be many local solutions. The solution actually found is often determined by the starting guess used. This makes the selection of an appropriate starting guess an important part of the solution process.

### Computation of Constraint Functions and Cost

The main task of the Core software is the computation of the constraint function vector `F(x)` and the scalar cost function `C(x)` as well as their partial derivatives `dF/dx` and `dC/dx`.

The components of `F` that correspond to the position, velocity and mass mismatches at the match points are computed by propagating the trajectory forward in time from the initial control node of a given leg to the match point time, and backward in time from the final control node of a given leg to the match point time. The match point mismatch is simply the difference between the forward propagation values and the corresponding backward propagation values:

```matlab
F_forward = [xf; yf; zf; vxf; vyf; vzf; mf];
F_back = [xb; yb; zb; vxb; vyb; vzb; mb];
F_mismatch = F_foward - F_back; % Equation (4)
```

As shown in Fig. 1, the trajectory is propagated from the control node to the match point as a sequence of twobody (conic) coasts separated by impulsive `delta_Vs` representing the low thrust acceleration over a thrust segment. The initial conditions (position, velocity and mass) for a given propagation from control node to match point are determined from the independent variables in `x`. The components of the thrust segment `delta_Vs` are also components of `x`  and are constrained by available low thrust acceleration at any particular point in the trajectory. If the propulsion is solar electric, the available electrical power for the low thrust engine is a function of the distance from the Sun and is computed by the Core solar power model. The user can choose from built-in models or define his own. For nuclear electric, the available electrical power is a user-supplied constant determined by the reactor. For either case, power requirements for spacecraft systems can be modeled and subtracted from the power available for thrust. The maximum low thrust force and propellant mass flow rate are determined as functions of available electrical power by the engine model. The Core has several built-in engine models and also allows the user to specify a custom engine model. The maximum thrust force as determined by the engine model is used to compute the maximum allowed `delta_V` for a given thrust segment (which becomes part of `F_UL` in Eq. (2). The propellant mass flow rate is used to decrement the spacecraft mass to account for fuel used during the segment.

The Core also allows modeling of solar sail trajectories. In this case, the thrust vector components for each segment represent the sail orientation vector (a unit vector perpendicular to the sail surface). For this case, the sail force is a function of the distance from the Sun and the sail orientation (as defined by the sail normal components for a particular segment). For solar sail, the mass flow rate is zero and spacecraft mass remains constant.

A model for launch vehicle performance computes the maximum mass a particular launch vehicle can deliver to a specified `V_inf`. The user can select from a list of built-in launch vehicles or define a new one. A spiral capture/escape model allows the spacecraft to either be captured to or escape from a circular orbit around a secondary body using a low thrust spiral. The fuel and time required for the spiral capture/escape are computed using an approximate analytic model. The Core also allows for an impulsive capture or escape to or from a specified elliptical orbit around a secondary body.
