---
layout: page
title:  walking-client
mathjax: true
permalink: /walking-client/
---

# The walking-client
The `walking-client` is an implementation of a predictive approach [1] to preview the duration and placement of coplanar contacts exploiting the OCRA framework and specifically for the iCub humanoid robot. Within a model-predictive control (MPC) framework, the problem is formulated as a *linearly constrained mixed-integer quadratic program* (MIQP) which allows the determination over a preview horizon, of the optimal changes in the base of support (BoS) of the robot with compatible Center of Mass (CoM) behaviour, subject to multiple constraints, while maximising balance and performance of a walking activity.

The following sections can be seen as an addendum to [1]. For more information, refer to the thesis itself.

# MIQP form of the walking MPC problem

The walking problem can be be seen as a compromise between CoM trajectory tracking and balance. By using the CoP distance from the boundaries of the BoS as an indicator of balance performance and CoM tracking error as the one for walking performance plus the addition of suitable constraints with integer and real variables, the walking problem writes:

$$
\begin{matrix}
\displaystyle \min_{\mathcal{X}} & \mathcal{X}^T \mathbf{H}_N \mathcal{X} + \mathbf{d}^T \mathcal{X}\\
\textrm{s.t.} & \mathbf{A} \mathcal{X} & \leq & \mathbf{f} \\
              & \xi_{k|k} & = & \xi_k \\
              & \xi_{k+j+1|k}          & = & \mathbf{Q} \xi_{k+j|k} + \mathbf{T} \mathcal{\xi}_{k+j+1|k} \\
              & (\mathbf{a}_{k+j|k}, \mathbf{b}_{k+j|k}) & \in & \mathbb{R}^2\times\mathbb{R}^2 \\
              & (\alpha_{k+j|k}, \beta_{k+j|k}) & \in & \{0,1\}^2 \times \{0,1\}^2\\
              & (\delta_{k+j|k}, \gamma_{k+j|k}) & \in & \{0,1\}^2 \times \{0,1\}^2\\
              & \mathbf{u}_{k+j|k} & \in & \mathbb{R}^2
\end{matrix}
$$



[1] Ibanez A. Ph.D. thesis: http://www.hal.inserm.fr/tel-01308723v2
