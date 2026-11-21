# Transformation Matrices - Robrot

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | Brazo1_Unido |
| $L_{2}$ | Brazo_2 |
| $L_{3}$ | Cremallera2 |
| $L_{4}$ | carril |
| $L_{5}$ | union |
| $L_{6}$ | Robrot_grip |
| $L_{7}$ | Robrot_grip_2 |
| $L_{8}$ | Robrot_guarda_2 |
| $L_{9}$ | Robrot_guarda |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | Revoluci_n_6 | revolute (rad) | $L_{0}$ | $L_{1}$ |
| $q_{2}$ | Revoluci_n_7 | revolute (rad) | $L_{1}$ | $L_{2}$ |
| $q_{3}$ | Corredera_9 | prismatic (m) | $L_{2}$ | $L_{3}$ |
| $q_{4}$ | Revoluci_n_12 | revolute (rad) | $L_{4}$ | $L_{5}$ |
| $q_{5}$ | Corredera_13 | prismatic (m) | $L_{4}$ | $L_{6}$ |
| $q_{6}$ | Corredera_14 | prismatic (m) | $L_{4}$ | $L_{7}$ |
| $q_{7}$ | Revoluci_n_15 | continuous (rad) | $L_{6}$ | $L_{8}$ |
| $q_{8}$ | Revoluci_n_17 | continuous (rad) | $L_{7}$ | $L_{9}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  +-- [revolute] Revoluci_n_6 (q1)
      L1: Brazo1_Unido
        +-- [revolute] Revoluci_n_7 (q2)
            L2: Brazo_2
              +-- [prismatic] Corredera_9 (q3)
                  L3: Cremallera2
                    +-- [fixed] R_gida_11
                        L4: carril
                          |-- [revolute] Revoluci_n_12 (q4)
                          |   L5: union
                          |-- [prismatic] Corredera_13 (q5)
                          |   L6: Robrot_grip
                          |     +-- [continuous] Revoluci_n_15 (q7)
                          |         L8: Robrot_guarda_2
                          +-- [prismatic] Corredera_14 (q6)
                              L7: Robrot_grip_2
                                +-- [continuous] Revoluci_n_17 (q8)
                                    L9: Robrot_guarda
```

## Transforms

## Revoluci_n_6

$L_{0}$ **base_link** -> $L_{1}$ **Brazo1_Unido** (revolute)
  Variable: $q_{1}$

- **origin xyz**: (-0.089343, 0.1591, 0.080111) m
- **origin rpy**: (-1.570796, 0, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [-1.570796, 1.570796] rad ([-90deg, 90deg])

### Local Transform

$T^{0}_{1}(q_{1}) = T_{fixed} \cdot R_{axis}(q_{1})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & -0.089343 \\
0 & 0 & 1 & 0.1591 \\
0 & -1 & 0 & 0.080111 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{1}) = \begin{bmatrix}
c_{1} & -s_{1} & 0 & 0 \\
s_{1} & c_{1} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revoluci_n_7

$L_{1}$ **Brazo1_Unido** -> $L_{2}$ **Brazo_2** (revolute)
  Variable: $q_{2}$

- **origin xyz**: (0.08265, -0.087, -0.1576) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [-1.570796, 1.570796] rad ([-90deg, 90deg])

### Local Transform

$$
T^{1}_{2}(q_{2}) = \begin{bmatrix}
c_{2} & -s_{2} & 0 & 0.08265 \\
s_{2} & c_{2} & 0 & -0.087 \\
0 & 0 & 1 & -0.1576 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Corredera_9

$L_{2}$ **Brazo_2** -> $L_{3}$ **Cremallera2** (prismatic)
  Variable: $q_{3}$

- **origin xyz**: (-0.129221, 0.020447, 0.430297) m
- **origin rpy**: (1.650854, 1.570796, 0) rad
- **axis**: (-1, 0, 0)
- **limits**: [0.005, 0.023] m

### Local Transform

$$
T^{2}_{3}(q_{3}) = \begin{bmatrix}
0 & 0.996797 & -0.079972 & -0.129221 - q_{3} \\
0 & -0.079972 & -0.996797 & 0.020447 \\
-1 & 0 & 0 & 0.430297 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## R_gida_11

$L_{3}$ **Cremallera2** -> $L_{4}$ **carril** (fixed)

- **origin xyz**: (0.395197, -0.157958, 0.000436) m
- **origin rpy**: (-3.141593, 0, 0) rad

### Local Transform

$$
T^{3}_{4} = \begin{bmatrix}
1 & 0 & 0 & 0.395197 \\
0 & -1 & 0 & -0.157958 \\
0 & 0 & -1 & 0.000436 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revoluci_n_12

$L_{4}$ **carril** -> $L_{5}$ **union** (revolute)
  Variable: $q_{4}$

- **origin xyz**: (-0.280783, 0.0426, 0.134217) m
- **origin rpy**: (-1.570796, -1.570796, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [0, 1.570796] rad ([0deg, 90deg])

### Local Transform

$T^{4}_{5}(q_{4}) = T_{fixed} \cdot R_{axis}(q_{4})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 1 & 0 & -0.280783 \\
0 & 0 & 1 & 0.0426 \\
1 & 0 & 0 & 0.134217 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{4}) = \begin{bmatrix}
c_{4} & -s_{4} & 0 & 0 \\
s_{4} & c_{4} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Corredera_13

$L_{4}$ **carril** -> $L_{6}$ **Robrot_grip** (prismatic)
  Variable: $q_{5}$

- **origin xyz**: (-0.122953, 0.0496, -0.270802) m
- **origin rpy**: (0, -1.570796, 0) rad
- **axis**: (1, 0, 0)

### Local Transform

$$
T^{4}_{6}(q_{5}) = \begin{bmatrix}
0 & 0 & -1 & -0.122953 + q_{5} \\
0 & 1 & 0 & 0.0496 \\
1 & 0 & 0 & -0.270802 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Corredera_14

$L_{4}$ **carril** -> $L_{7}$ **Robrot_grip_2** (prismatic)
  Variable: $q_{6}$

- **origin xyz**: (0.005709, 0.0496, 0.401396) m
- **origin rpy**: (0, 1.570796, 0) rad
- **axis**: (-1, 0, 0)

### Local Transform

$$
T^{4}_{7}(q_{6}) = \begin{bmatrix}
0 & 0 & 1 & 0.005709 - q_{6} \\
0 & 1 & 0 & 0.0496 \\
-1 & 0 & 0 & 0.401396 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revoluci_n_15

$L_{6}$ **Robrot_grip** -> $L_{8}$ **Robrot_guarda_2** (continuous)
  Variable: $q_{7}$

- **origin xyz**: (0.011376, -0.004, -0.16773) m
- **origin rpy**: (-1.570796, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{6}_{8}(q_{7}) = T_{fixed} \cdot R_{axis}(q_{7})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0.011376 \\
0 & 0 & 1 & -0.004 \\
0 & -1 & 0 & -0.16773 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{7}) = \begin{bmatrix}
c_{7} & -s_{7} & 0 & 0 \\
s_{7} & c_{7} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revoluci_n_17

$L_{7}$ **Robrot_grip_2** -> $L_{9}$ **Robrot_guarda** (continuous)
  Variable: $q_{8}$

- **origin xyz**: (0.606532, -0.004, 0.098771) m
- **origin rpy**: (1.570796, 0, -3.141593) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{7}_{9}(q_{8}) = T_{fixed} \cdot R_{axis}(q_{8})$ where:

$$
T_{fixed} = \begin{bmatrix}
-1 & 0 & 0 & 0.606532 \\
0 & 0 & 1 & -0.004 \\
0 & 1 & 0 & 0.098771 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{8}) = \begin{bmatrix}
c_{8} & -s_{8} & 0 & 0 \\
s_{8} & c_{8} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

$$T^{0}_{2} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2})\quad (L_0 \to L_{2}: \text{Brazo_2})$$

$$T^{0}_{3} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3})\quad (L_0 \to L_{3}: \text{Cremallera2})$$

$$T^{0}_{4} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}\quad (L_0 \to L_{4}: \text{carril})$$

$$T^{0}_{5} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{4})\quad (L_0 \to L_{5}: \text{union})$$

$$T^{0}_{6} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4} \cdot T^{4}_{6}(q_{5})\quad (L_0 \to L_{6}: \text{Robrot_grip})$$

$$T^{0}_{7} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4} \cdot T^{4}_{7}(q_{6})\quad (L_0 \to L_{7}: \text{Robrot_grip_2})$$

$$T^{0}_{8} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4} \cdot T^{4}_{6}(q_{5}) \cdot T^{6}_{8}(q_{7})\quad (L_0 \to L_{8}: \text{Robrot_guarda_2})$$

$$T^{0}_{9} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4} \cdot T^{4}_{7}(q_{6}) \cdot T^{7}_{9}(q_{8})\quad (L_0 \to L_{9}: \text{Robrot_guarda})$$

