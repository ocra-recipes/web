---
layout: page
title:  walking-client
mathjax: true
permalink: /walking-client/
---

# The walking-client
The `walking-client` is an implementation of a predictive approach [1] to preview the duration and placement of coplanar contacts exploiting the OCRA framework and specifically for the iCub humanoid robot. Within a model-predictive control (MPC) framework, the problem is formulated as a *linearly constrained mixed-integer quadratic program* (MIQP) which allows the determination over a preview horizon, of the optimal changes in the base of support (BoS) of the robot with compatible Center of Mass (CoM) behaviour, subject to multiple constraints, while maximising balance and performance of a walking activity.

The following sections can be seen as an addendum to [1] which give a little bit more of detail on the different sections. For more information, refer to the thesis itself.

# MIQP form of the walking MPC problem

The walking problem can be be seen as a compromise between CoM trajectory tracking and balance. By using the CoP distance from the boundaries of the BoS as an indicator of balance performance and CoM tracking error as the one for walking performance plus the addition of suitable constraints with integer and real variables, the walking problem writes:

$$
\begin{equation} \label{eq:miqpEquation}
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
\end{equation}
$$

The MIQP canonical form of the cost function originates from the following cost function:

$$
J_k = \omega_b\sum_{j=1}^{N} || \mathbf{p}_{k+j|k} - \mathbf{r}_{k+j|k} ||^2 +  \omega_w \sum_{j=1}^{N} ||\mathbf{S}(\hat{\mathbf{h}}_{k+j|k} - \hat{\mathbf{h}}^r_{k+j|k}) ||^2
$$

which represents the formerly described compromise between the walking ($J_{w_k}$) and balance ($J_{b_k}$) performances, i.e.

$$
J_k = \omega_b J_{b_k} + \omega_w J_{w_k} + q
$$

Let's work out a bit more Equation \ref{eq:miqpEquation} before going into the details of the constraints.

## Cost function
The global cost function (without regularization terms) can be written in matrix form as:

$$
\begin{equation} \label{eq:costFuncMatrix}
(\mathbf{H}^r - \mathbf{H})^T \mathbf{S}_w(\mathbf{H}^r - \mathbf{H}) + (\mathbf{P}_N - \mathbf{R}_N)^T \mathbf{N}_b (\mathbf{P}_N - \mathbf{R}_N)
\end{equation}
$$

Where, in the walking performance cost, $\mathbf{H}$ is the vector of predicted CoM outputs in the preview window of size $N$, while $\mathbf{H}^r$ is the vector of CoM state references in the preview window. On the other hand, $\mathbf{P}$ is the vector of predicted CoP outputs and $\mathbf{R}$, the vector of predicted BoS centers.

#### Predicted CoM outputs
Given the system state $xi$ (more on this in [1] Sections 5.1.2, 5.2) and input $\mathcal{X}$, we can predict the CoM outputs in a preview window ($\mathbf{H}$) from the state propagation:

$$
\begin{align}
\xi_{k+j+1|k} & = \mathbf{Q}\xi_{k+j|k} + \mathbf{T}\mathcal{X}_{k+j+1|k}\\
\end{align}
$$

Where

$$
\mathbf{Q} = \left[\begin{array}{cc}
\mathbf{0}_{10\times10} & \mathbf{0}_{10\times6}\\
\mathbf{0}_{6\times10} & \mathbf{A_h}_{6\times6}
\end{array}\right]
$$

and the relationship between the system and CoM state, i.e.

$$
\begin{equation}
\hat{\mathbf{h}}_{k+j+1|k} = \mathbf{C}_h \xi_{k+j+1|k}
\end{equation}
$$

By iteratively applying the previous system of equations we arrive to the following compact matrix expression:

$$
\mathbf{H}_{k,N} = \mathbf{P}_H \mathbf{\xi}_k + \mathbf{R}_H \mathcal{X}_{k,N}
$$

Where

$$
\begin{align}
\mathbf{P}_H = \left[\begin{array}{c}
\mathbf{C}_H \mathbf{Q} \\
\vdots\\
\mathbf{C}_H \mathbf{Q}^N
\end{array}\right]
\end{align}
$$

$$
\mathbf{R}_H = \left[\begin{array}{cccc}
\mathbf{C}_H\mathbf{T}               &   0                          &  \cdots   &   0 \\
\mathbf{C}_H\mathbf{Q}\mathbf{T}   &   \mathbf{C}_H\mathbf{T}   &  \cdots   &   0 \\
\vdots                                 & \vdots                       & \ddots    &  \vdots \\
\mathbf{C}_H\mathbf{Q}^{N-1}\mathbf{T} & \mathbf{C}_H\mathbf{Q}^{N-2}\mathbf{T} & \cdots & \mathbf{C}_H\mathbf{T}
\end{array}\right]
$$

[1] Ibanez A. Ph.D. thesis: http://www.hal.inserm.fr/tel-01308723v2
