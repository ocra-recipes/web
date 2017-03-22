---
layout: page
title:  walking-client
mathjax: true
permalink: /walking-client/
---

# The walking-client
The `walking-client` is an implementation of a predictive approach [1] to preview the duration and placement of coplanar feet contacts of the iCub humanoid robot exploiting the OCRA framework. Within a model-predictive control (MPC) framework, the problem is formulated as a *linearly constrained mixed-integer quadratic program* (MIQP) which allows the determination over a preview horizon, of the optimal changes in the base of support (BoS) of the robot with compatible Center of Mass (CoM) behaviour, subject to multiple constraints, while maximising balance and performance of a walking activity.

> The following sections can be seen as an addendum to [1] which gives a detailed insight of the MIQP formulation. It assumes that the reader has gone through [1] (or at least Chapter 5) and needs more details for understanding the nuances of the client `walking-client`. For more theoretical and complementary information, refer to the thesis itself.

# MIQP form of the walking MPC problem

The walking problem can be be seen as a compromise between CoM trajectory tracking and balance. By using the CoP distance from the boundaries of the BoS as the indicator for balance performance and CoM tracking error as the one for walking performance, plus the addition of suitable constraints with integer and real variables, the walking problem writes:

$$
\begin{equation} \label{eq:miqpEquation}
\begin{matrix}
\displaystyle \min_{\mathcal{X}} & \mathcal{X}^T \mathbf{H}_N \mathcal{X} + \mathbf{d}^T \mathcal{X}\\
\textrm{s.t.} & \mathbf{A} \mathcal{X} & \leq & \mathbf{f} \\
              & \xi_{k|k} & = & \xi_k \\
              & \xi_{k+j+1|k}          & = & \mathbf{Q} \xi_{k+j|k} + \mathbf{T} \mathcal{X}_{k+j+1|k} \\
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

Where $q$ are regularization terms. More details later on.

Let's work out a bit more Equation \ref{eq:miqpEquation} before moving to the details of the constraints.

## Cost function
The global cost function (without regularization terms) can be written in matrix form as:

$$
\begin{equation} \label{eq:costFuncMatrix}
(\mathbf{H}^r - \mathbf{H}_{k,N})^T \mathbf{S}_w(\mathbf{H}^r - \mathbf{H}_{k,N}) + (\mathbf{P}_{k,N} - \mathbf{R}_{k,N})^T \mathbf{N}_b (\mathbf{P}_{k,N} - \mathbf{R}_{k,N})
\end{equation}
$$

Where, in the walking performance cost, $$\mathbf{H}_{k,N}$$ is the vector of predicted CoM outputs in a preview window of size $N$, while $$\mathbf{H}^r$$ is the vector of CoM state references in the same window. On the other hand, $$\mathbf{P}_{k,N}$$ is the vector of predicted CoP outputs and $$\mathbf{R}_{k,N}$$, the vector of predicted BoS centers.

### Predicted CoM outputs
Given the system state $\xi$ (more on this in [1] Sections 5.1.2, 5.2) and input $\mathcal{X}$, we can predict the CoM outputs in a preview window ($\mathbf{H}_{k,N}$) from the state propagation:

$$
\begin{align}\label{eq:previewModel}
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

$$
\mathbf{T} = \left[\begin{array}{cc}
\mathbf{I}_{10\times10} & \mathbf{0}_{10\times2}\\
\mathbf{0}_{6\times10} & \mathbf{B_h}_{6\times2}
\end{array}\right]
$$

and the relationship between the system and CoM state, i.e.

$$
\begin{equation}
\hat{\mathbf{h}}_{k+j+1|k} = \mathbf{C}_h \xi_{k+j+1|k}
\end{equation}
$$

Then by iteratively applying the previous system of equations, we arrive to the following compact matrix expression:

$$
\begin{equation}\label{eq:comPrediction}
\mathbf{H}_{k,N} = \mathbf{P}_H \mathbf{\xi}_k + \mathbf{R}_H \mathcal{X}_{k,N}
\end{equation}
$$

Where:

$$
\begin{equation}
\mathbf{P}_H = \left[\begin{array}{c}
\mathbf{C}_H \mathbf{Q} \\
\vdots\\
\mathbf{C}_H \mathbf{Q}^N
\end{array}\right]
\end{equation}
$$

$$
\begin{equation}
\mathbf{R}_H = \left[\begin{array}{cccc}
\mathbf{C}_H\mathbf{T}               &   0                          &  \cdots   &   0 \\
\mathbf{C}_H\mathbf{Q}\mathbf{T}   &   \mathbf{C}_H\mathbf{T}   &  \cdots   &   0 \\
\vdots                                 & \vdots                       & \ddots    &  \vdots \\
\mathbf{C}_H\mathbf{Q}^{N-1}\mathbf{T} & \mathbf{C}_H\mathbf{Q}^{N-2}\mathbf{T} & \cdots & \mathbf{C}_H\mathbf{T}
\end{array}\right]
\end{equation}
$$

We will call matrices like $\mathbf{P}_H$ and $\mathbf{R}_H$, *preview state* and *preview input* matrices respectively.

### Predicted CoP outputs
We need to proceed in a similar fashion as before to obtain an expression for the predicted CoP outputs in the preview window. So,
again, keeping in mind that:

$$
\begin{equation}
\mathbf{P}_{k,N} = \left\{ \mathbf{p}_{k+1|k}, \mathbf{p}_{k+2|k}, \dots, \mathbf{p}_{k+N|k} \right\}
\end{equation}
$$

And that in this case:

$$
\begin{align}
\xi_{k+j+1|k} & = \mathbf{Q}\xi_{k+j|k} + \mathbf{T}\mathcal{X}_{k+j+1|k}\\
\mathbf{p}_{h+j+1|k} &= \mathbf{C}_P \xi_{k+j+1|k}
\end{align}
$$

Where, assuming a simplified Linear Inverted Pendulum model of the robot and constant CoM height $c_z$:

$$
\mathbf{C}_P = \left[
\begin{array}{ccccc}
\mathbf{0}_{2\times10} & \mathbf{I}_{2\times2} & \mathbf{0}_{2\times2} & -\frac{c_z}{g}\mathbf{I}_{2\times2}
\end{array}\right]
$$

We can thus obtain a compact expression for $\mathbf{P}_{k,N}$ as:

$$
\begin{equation}\label{eq:copPrediction}
\mathbf{P}_{k,N} = \mathbf{P}_P \xi_k + \mathbf{R}_P \mathcal{X}_{k,N}
\end{equation}
$$

Where $\mathbf{P}_P$ and $\mathbf{R}_P$ have the same structure as $\mathbf{P}_H$ and $\mathbf{R}_H$ but using $\mathbf{C}_P$ instead of $\mathbf{C}_H$.

### Predicted BoS outputs
Once more we look for an expression for the predicted BoS outputs in the preview window, but this time:

$$
\begin{align}
\xi_{k+j+1|k} & = \mathbf{Q}\xi_{k+j|k} + \mathbf{T}\mathcal{X}_{k+j+1|k}\\
\mathbf{r}_{k+j+1|k} &= \mathbf{C}_B \xi_{k+j+1|k}
\end{align}
$$

Where:

$$
\mathbf{C}_B = \frac{1}{2} \left[
\begin{array}{ccc}
\mathbf{I}_{2\times2} & \mathbf{I}_{2\times2} & \mathbf{0}_{2\times12}
\end{array}\right]
$$

We can again obtain a compact expression for the predicted BoS outputs:

$$
\begin{equation}  \label{eq:bosPrediction}
\mathbf{R}_{k,N} = \mathbf{P}_B \xi_k + \mathbf{R}_B \mathcal{X}_{k,N}
\end{equation}
$$

### Expansion of the cost function
By replacing the previous CoM (\ref{eq:comPrediction}), CoP (\ref{eq:copPrediction}) and BoS (\ref{eq:bosPrediction}) predicted outputs in the matrix form of the cost function (\ref{eq:costFuncMatrix}) we get:

For the first part of the cost function $$(\mathbf{H}^r - \mathbf{H}_{k,N})^T \mathbf{S}_w(\mathbf{H}^r - \mathbf{H}_{k,N})$$ given the symmetry of some resulting terms and getting rid of some subscripts to simplify the writing:

$$
\begin{align*}
&(\mathbf{H}^r - \mathbf{P}_H\xi_k - \mathbf{R}_H\mathcal{X})^T \mathbf{S}_w(\mathbf{H}^r - \mathbf{P}_H \xi_k - \mathbf{R}_H \mathcal{X})\\
&=(\mathbf{H}^r - \mathbf{P}_H\xi_k)^T\mathbf{S}_w(\mathbf{H}^r - \mathbf{P}_H\xi_k) - (\mathbf{H}^r - \mathbf{P}_H\xi_k)^T\mathbf{S}_w\mathbf{R}_H\mathcal{X} - (\mathbf{R}_H\mathcal{X})^T\mathbf{S}_w(\mathbf{H}^r - \mathbf{P}_H\xi_K) + (\mathbf{R}_H \mathcal{X})^T\mathbf{S}_w\mathbf{R}_H\mathcal{X}\\
&=(\mathbf{H}^r - \mathbf{P}_H\xi_k)^T\mathbf{S}_w(\mathbf{H}^r - \mathbf{P}_H\xi_k) - 2(\mathbf{H}^r - \mathbf{P}_H\xi_k)^T\mathbf{S}_w\mathbf{R}_H\mathcal{X} + (\mathbf{R}_H \mathcal{X})^T\mathbf{S}_w\mathbf{R}_H\mathcal{X}
\end{align*}
$$

$$
\begin{equation}\label{eq:firstTermCompact}
= (\mathbf{H}^r - \mathbf{P}_H\xi_k)^T\mathbf{S}_w(\mathbf{H}^r - \mathbf{P}_H\xi_k) - 2(\mathbf{H}^r - \mathbf{P}_H\xi_k)^T\mathbf{S}_w\mathbf{R}_H\mathcal{X} + \mathcal{X}^T\mathbf{R}_H^T\mathbf{S}_w\mathbf{R}_H\mathcal{X}
\end{equation}
$$

The first term in (\ref{eq:firstTermCompact}) is not a function of $\mathcal{X}$ and therefore won't make part of the final cost function.

Expanding the second part of the original cost function, i.e. $$(\mathbf{P}_{k,N} - \mathbf{R}_{k,N})^T \mathbf{N}_b (\mathbf{P}_{k,N} - \mathbf{R}_{k,N})$$ and again exploiting the symmetry of some resulting terms:

$$
\begin{align*}
&((\mathbf{P}_P\xi_k + \mathbf{R}_P \mathcal{X}) - (\mathbf{P}_B \xi_k + \mathbf{R}_B\mathcal{X}))^T\mathbf{N}_b((\mathbf{P}_P\xi_k + \mathbf{R}_P\mathcal{X}) - (\mathbf{P}_B \xi_k + \mathbf{R}_B\mathcal{X}))\\
&=((\mathbf{P}_P - \mathbf{P}_B)\xi_k + (\mathbf{R}_P - \mathbf{R}_B)\mathcal{X})^T \mathbf{N}_b ((\mathbf{P}_P - \mathbf{P}_B)\xi_k + (\mathbf{R}_P - \mathbf{R}_B)\mathcal{X})\\
&=[(\mathbf{P}_P - \mathbf{P}_B)\xi_k]^T\mathbf{N}_b(\mathbf{P}_P-\mathbf{P}_B)\xi_k + [(\mathbf{P}_P - \mathbf{P}_B)\xi_k]^T\mathbf{N}_b(\mathbf{R}_P - \mathbf{R}_B)\mathcal{X} + \\
& \dots[(\mathbf{R}_P-\mathbf{R}_B)\mathcal{X}]^T\mathbf{N}_b(\mathbf{P}_P - \mathbf{P}_B)\xi_k + [(\mathbf{R}_P - \mathbf{R}_B)\mathcal{X}]^T\mathbf{N}_b(\mathbf{R}_P - \mathbf{R}_B)\mathcal{X}
\end{align*}
$$

$$
\begin{equation} \label{eq:secontTermCompact}
= 2((\mathbf{P}_P - \mathbf{P}_B)\xi_k)^T\mathbf{N}_b(\mathbf{R}_P - \mathbf{R}_B)\mathcal{X} + \mathcal{X}^T(\mathbf{R}_P - \mathbf{R}_B)^T \mathbf{N}_b(\mathbf{R}_P - \mathbf{R}_B)\mathcal{X}
\end{equation}
$$

Adding up (\ref{eq:firstTermCompact}) and (\ref{eq:secontTermCompact}) (only those terms which are function of $\mathcal{X}$) we get:

$$
\begin{align}
&-2(\mathbf{H}^r - \mathbf{P}_H \xi_k)^T\mathbf{S}_w\mathbf{R}_H\mathcal{X} + 2[(\mathbf{P}_P - \mathbf{P}_B)\xi_k]^T\mathbf{N}_b(\mathbf{R}_P - \mathbf{R}_B)\mathcal{X} + \dots\\
&\mathcal{X}^T\mathbf{R}_H^T\mathbf{S}_w\mathbf{R}_w\mathcal{X} + \mathcal{X}^T(\mathbf{R}_P - \mathbf{R}_B)^T\mathbf{N}_b(\mathbf{R}_P - \mathbf{R}_B)\mathcal{X}
\end{align}
$$

Which gives the final expression for the cost function:

$$
J_k = \mathcal{X}^T\mathbf{H}_N\mathcal{X} + \mathbf{d}^T\mathcal{X}
$$

Where:

$$
\begin{align}
\mathbf{H}_N&: \mathbf{R}_H^T\mathbf{S}_w\mathbf{R}_H + (\mathbf{R}_P - \mathbf{R}_B)^T\mathbf{N}_b(\mathbf{R}_P - \mathbf{R}_B)\\
\mathbf{d}^T&: -2(\mathbf{H}^r - \mathbf{P}_H\xi_k)^T\mathbf{S}_w\mathbf{R}_H + 2[(\mathbf{P}_P - \mathbf{P}_B)\xi_k]^T\mathbf{N}_b(\mathbf{R}_P - \mathbf{R}_B)
\end{align}
$$

## Constraints
The constraints of the MIQP problem (\ref{eq:miqpEquation}) **at this moment** contain three types of constraints:
- Shape constraints
- Admissibility constraints
- CoP constraints

All constraints are linear. The following sections do not provide an explanation of what each constraint is, but shows only the corresponding matrix expressions later used in `walking-client`. For further information, refer to [1]-Appendix B.

### Shape Constraints
These consist at time step $i$ of:

**Bounding**:

$$
\begin{equation} \label{eq:bounding}
\mathbf{C}_i^B \xi_i \leq \mathbf{d}^B
\end{equation}
$$

**Constancy**:

$$
\begin{equation} \label{eq:constancy}
\mathbf{C}_i^C \xi_i + \mathbf{C}_{ii}^C \xi_{i+1} \leq \mathbf{d}^C
\end{equation}
$$

**Sequentiality**:

$$
\begin{equation} \label{eq:sequentiality}
\mathbf{C}_i^S \xi_i \leq \mathbf{d}^S
\end{equation}
$$

These constraints have a similar structure, depending either on the the current state only or also on the next system state. Therefore Shape Constraints at time step $i$ can be expressed in a compact form as:

$$
\begin{equation}\label{eq:shapeCompact}
\mathbf{A}_{cl}^{S} \xi_i + \mathbf{A}_{cr}^{S}\xi_{i+1} \leq \mathbf{f}_c^S
\end{equation}
$$

#### Bounding Constraints Matrices
In Eq. (\ref{eq:bounding})

$$
\begin{equation}
\mathbf{C}_i =
\left[\begin{array}{ccccc}
-1 & 0 & 1 & 0 & 0_{1\times8}\\
 0 &-1 & 0 & 1 & 0_{1\times8}
\end{array}\right]
\end{equation}
$$

and

$$
\mathbf{d} = \left[\begin{array}{c}
0\\
0
\end{array}\right]
$$

#### Constancy Constraints Matrices
In Eq. (\ref{eq:constancy})

$$
\mathbf{C}_{i} = \left[\begin{array}{ccccc}
-1 & 0 & 0 & 0 & 0_{1\times12}\\
 1 & 0 & 0 & 0 & 0_{1\times12}\\
 0 & 0 &-1 & 0 & 0_{1\times12}\\
 0 & 0 & 1 & 0 & 0_{1\times12}\\
 0 &-1 & 0 & 0 & 0_{1\times12}\\
 0 & 1 & 0 & 0 & 0_{1\times12}\\
 0 & 0 & 0 &-1 & 0_{1\times12}\\
 0 & 0 & 0 & 1 & 0_{1\times12}
\end{array}\right]
$$

$$
\mathbf{C}_{ii} = \left[\begin{array}{ccccc}
 1 & 0 & 0 & 0 & -s_x & 0    &  0   & 0    & 0_{1\times8}\\
-1 & 0 & 0 & 0 & -s_x & 0    &  0   & 0    & 0_{1\times8}\\
 0 & 0 & 1 & 0 &    0 & 0    & -s_x & 0    & 0_{1\times8}\\
 0 & 0 &-1 & 0 &    0 & 0    & -s_x & 0    & 0_{1\times8}\\
 0 & 1 & 0 & 0 &    0 & -s_y &  0   & 0    & 0_{1\times8}\\
 0 &-1 & 0 & 0 &    0 & -s_y &  0   & 0    & 0_{1\times8}\\
 0 & 0 & 0 & 1 &    0 & 0    &  0   & -s_y & 0_{1\times8}\\
 0 & 0 & 0 &-1 &    0 & 0    &  0   & -s_y & 0_{1\times8}
\end{array}\right]
$$

And

$$
\mathbf{d} = \left[\begin{array}{c}
0\\
\vdots\\
0
\end{array}\right]
$$

#### Sequentiality Matrices
In Eq. (\ref{eq:sequentiality})

$$
\mathbf{C}_{i} = \left[\begin{array}{cccccc}
0_{1\times4} & 1 & 0 & 1 & 0 & 0_{1\times8}\\
0_{1\times4} & 0 & 1 & 0 & 1 & 0_{1\times8}
\end{array}\right]
$$

$$
\mathbf{d} = \left[\begin{array}{c}
1\\
1
\end{array}\right]
$$

### Admissibility Constraints
These consist at time step $i$ of:

**Single and Double Support Alternation**

$$
\begin{equation} \label{eq:SDSAlternation}
\mathbf{C}_i^{SD} \xi_i + \mathbf{C}_{ii}^{SD} \xi_{i+1} \leq \mathbf{d}^{SD}
\end{equation}
$$

**Single Support**

$$
\begin{equation} \label{eq:singleSupport}
\mathbf{C}_i^{SS} \xi_i \leq \mathbf{d}^{SS}
\end{equation}
$$

**Contact Configuration History**

$$
\begin{equation} \label{eq:contactConfigHistory}
\mathbf{C}_i^{CH} \xi_i + \mathbf{C}_{ii}^{CH} \xi_{i+1} \leq \mathbf{d}^{CH}
\end{equation}
$$

**Contact Configuration Enforcement**

$$
\begin{equation} \label{eq:contactConfigEnforcement}
\mathbf{C}_i^{CE} \xi_i \leq \mathbf{d}^{CE}
\end{equation}
$$

These constraints have a similar structure, depending either on the the current state only or also on the next system state. Therefore Admissibility Constraints at time step $i$ can be expressed in a compact form as:

$$
\begin{equation} \label{eq:admissibilityCompact}
\mathbf{A}_{cl}^{A} \xi_i + \mathbf{A}_{cr}^{A}\xi_{i+1} \leq \mathbf{f}_c^{A}
\end{equation}
$$

#### Single And Double Support Alternation Matrices
In Eq. (\ref{eq:SDSAlternation})

$$
\mathbf{C}_{ii} = \left[\begin{array}{ccccccc}
0_{1\times4} & 1 & 0 & 1 & 0_{1\times2} & 1 & 0_{1\times6}\\
0_{1\times4} & 1 & 0 & 1 & 0_{1\times2} &-1 & 0_{1\times6}\\
0_{1\times4} &-1 & 0 &-1 & 0_{1\times2} & 1 & 0_{1\times6}\\
0_{1\times4} &-1 & 0 &-1 & 0_{1\times2} &-1 & 0_{1\times6}\\
\end{array}\right]
$$

$$
\mathbf{C}_{i} = \left[\begin{array}{ccc}
0_{1\times9} & 1 & 0_{1\times6}\\
0_{1\times9} &-1 & 0_{1\times6}\\
0_{1\times9} &-1 & 0_{1\times6}\\
0_{1\times9} & 1 & 0_{1\times6}
\end{array}\right]
$$

And

$$
\mathbf{d} = \left[\begin{array}{c}
2\\
0\\
0\\
0
\end{array}\right]
$$

#### Single Support Matrices
In Eq. (\ref{eq:singleSupport})

$$
\mathbf{C}_{i} = \left[\begin{array}{ccccccccc}
 1 & 0 &-1 & 0 & 0 & 0 & 0 & -s_x & 0_{1\times6}\\
-1 & 0 & 1 & 0 & 0 & 0 & 0 & -s_x & 0_{1\times6}\\
 0 & 1 & 0 &-1 & 0 & 0 & 0 & -s_y & 0_{1\times6}\\
 0 &-1 & 0 & 1 & 0 & 0 & 0 & -s_y & 0_{1\times6}\\
\end{array}\right]
$$

and

$$
\mathbf{d} = \left[\begin{array}{c}
0\\
0\\
0\\
0
\end{array}\right]
$$

#### Contact Configuration History Matrices
In Eq. (\ref{eq:contactConfigHistory})

$$
\mathbf{C}_{ii} = \left[\begin{array}{cccc}
0_{1\times8} & 1 & 0 & 0_{1\times6}\\
0_{1\times8} &-1 & 0 & 0_{1\times6}\\
\end{array}\right]
$$

$$
\mathbf{C}_{i} = \left[\begin{array}{cccc}
0_{1\times8} &-1 & 1 & 0_{1\times6}\\
0_{1\times8} & 1 & 1 & 0_{1\times6}\\
\end{array}\right]
$$

and

$$
\mathbf{d} = \left[\begin{array}{c}
1\\
1
\end{array}\right]
$$

#### Contact Configuration Enforcement Matrices
In Eq. (\ref{eq:contactConfigEnforcement})

$$
\mathbf{C}_{i} = \left[\begin{array}{ccccccc}
 0_{1\times4} & -1 & -1 & 0 & 0 & 1 & 0_{1\times7}\\
 0_{1\times4} & -1 &  1 & 0 & 0 & 1 & 0_{1\times7}\\
 0_{1\times4} &  0 &  0 & 1 &-1 & 1 & 0_{1\times7}\\
 0_{1\times4} &  0 &  0 &-1 & 1 & 1 & 0_{1\times7}\\
 0_{1\times4} &  1 &  0 & 0 &-1 &-1 & 0_{1\times7}\\
 0_{1\times4} & -1 &  0 & 0 & 1 &-1 & 0_{1\times7}\\
 0_{1\times4} &  0 & -1 & 1 & 0 &-1 & 0_{1\times7}\\
 0_{1\times4} &  0 &  1 &-1 & 0 &-1 & 0_{1\times7}
\end{array}\right]
$$

$$
\mathbf{d} = \left[\begin{array}{c}
1\\
1\\
1\\
1\\
0\\
0\\
0\\
0
\end{array}\right]
$$

### Shape and Admissibility Constraints in the Preview Window
Recalling the preview model (\ref{eq:previewModel}) we can obtain an expression for the constraints matrices in the preview window as done for the CoM, CoP and BoS in previous sections. Thus, stacking inequalities (\ref{eq:shapeCompact}) and (\ref{eq:admissibilityCompact}) by denoting:

$$
\mathbf{A}_{cl} = \left[ \begin{array}{c}
\mathbf{A}_{cl}^{S}\\
\mathbf{A}_{cl}^{A}
\end{array}\right]
$$

$$
\mathbf{A}_{cr} = \left[ \begin{array}{c}
\mathbf{A}_{cr}^{S}\\
\mathbf{A}_{cr}^{A}
\end{array}\right]
$$

$$
\mathbf{f}_{c} = \left[ \begin{array}{c}
\mathbf{f}_{c}^{S}\\
\mathbf{f}_{c}^{A}
\end{array}\right]
$$

We have shape and admissibility constraints at time $i$ in compact form as:

$$
\begin{equation} \label{eq:shapeAdmissCompact}
\mathbf{A}_{cl}\xi_i + \mathbf{A}_{cr}\xi_{i+1} \leq \mathbf{f}_c
\end{equation}
$$

We need however to write these constraints in the preview window and in terms of the optimization variable $\mathcal{X}$ as $$\mathbf{A}\mathcal{X}\leq\mathbf{f}$$

Thus, by iteratively applying Eq.(\ref{eq:shapeAdmissCompact}) for $i\in[k,N-1]$ we get:

$$
\begin{equation}
\mathbf{A} = \left[ \begin{array}{ccccc}
\mathbf{A}_{cr}\mathbf{Q}^0\mathbf{T} & & & & \\
\mathbf{A}_{cl}\mathbf{Q}^0\mathbf{T} + \mathbf{A}_{cr}\mathbf{Q}^1 \mathbf{T} & \mathbf{A}_{cr}\mathbf{Q}^0\mathbf{T} & & & \\
\mathbf{A}_{cl}\mathbf{Q}^1\mathbf{T} + \mathbf{A}_{cr}\mathbf{Q}^2 \mathbf{T} & \mathbf{A}_{cl}\mathbf{Q}^0\mathbf{T} + \mathbf{A}_{cr}\mathbf{Q}^1\mathbf{T} & \mathbf{A}_{cr}\mathbf{Q}^0\mathbf{T} & & \\
\vdots & \vdots & \vdots & \ddots & \\
\mathbf{A}_{cl}\mathbf{Q}^{N-2}\mathbf{T} + \mathbf{A}_{cr}\mathbf{Q}^{N-1}\mathbf{T} & \mathbf{A}_{cl}\mathbf{Q}^{N-3}\mathbf{T} + \mathbf{A}_{cr}\mathbf{Q}^{N-2}\mathbf{T} & \dots & & \mathbf{A}_{cr}\mathbf{Q}^0\mathbf{T}
\end{array}\right]
\end{equation}
$$

While:

$$
\begin{equation}
\mathbf{f} =
\left[\begin{array}{c}
\mathbf{f}_c\\
\mathbf{f}_c\\
\vdots\\
\mathbf{f}_c
\end{array}\right] -
\left[\begin{array}{c}
\mathbf{A}_{cl}\mathbf{Q}^0 + \mathbf{A}_{cr}\mathbf{Q}^1\\
\mathbf{A}_{cl}\mathbf{Q}^1 + \mathbf{A}_{cr}\mathbf{Q}^2\\
\vdots\\
\mathbf{A}_{cl}\mathbf{Q}^{N-1} + \mathbf{A}_{cr}\mathbf{Q}^N
\end{array}\right] \xi_k
\end{equation}
$$

### Base of Support Constraints
In this approach the convex hull is overestimated by its bounding box and defined by the minimum and maximum points as shown in the next image:

![Bounding Box]({{ site.url }}/static/img/bounding-box.svg){:class="img-responsive"}

These constraints write in terms of the CoP $\mathbf{p}$:

$$
\begin{equation}
\left[\begin{array}{cc}
-1  &  0\\
1  &  0\\
0  & -1\\
0  &  1
\end{array}\right] \mathbf{p}
\leq
\left[\begin{array}{c}
-x_{\text{min}}\\
x_{\text{max}}\\
-y_{\text{min}}\\
y_{\text{max}}
\end{array}\right]
\end{equation}
$$

or $\mathbf{A}_b\mathbf{p}\leq\mathbf{b}$, and in terms of the system state as:

$$
\begin{equation} \label{eq:bboxConstraints}
\mathbf{C}_i \xi \leq \mathbf{f}
\end{equation}
$$

Where:

$$
\begin{equation}
\mathbf{C}_i=
\left[\begin{array}{cc}
\mathbf{0}_{10\times10} & \\
& \mathbf{A}_b \mathbf{C}_p
\end{array}\right]
\end{equation}
$$

and

$$
\mathbf{f}=\left[\begin{array}{c}
\mathbf{0} \\
\mathbf{b}
\end{array}\right]
$$

Then, in a preview window, given preview model (\ref{eq:previewModel}) and iteratively applying Eq.(\ref{eq:bboxConstraints}) we get the following set of constraints:

$$
\mathbf{A}_{\text{BoS}} \mathcal{X} \leq \mathbf{f}_{\text{BoS}}
$$

Where:

$$
\begin{equation}
\mathbf{A}_{\text{BoS}} = \left[\begin{array}{cccc}
\mathbf{C}_i\mathbf{T}               &   0                          &  \cdots   &   0 \\
\mathbf{C}_i\mathbf{Q}\mathbf{T}   &   \mathbf{C}_i\mathbf{T}   &  \cdots   &   0 \\
\vdots                                 & \vdots                       & \ddots    &  \vdots \\
\mathbf{C}_i\mathbf{Q}^{N-1}\mathbf{T} & \mathbf{C}_i\mathbf{Q}^{N-2}\mathbf{T} & \cdots & \mathbf{C}_i\mathbf{T}
\end{array}\right]
\end{equation}
$$

[1] Ibanez A. Ph.D. [thesis](http://www.hal.inserm.fr/tel-01308723v2)
