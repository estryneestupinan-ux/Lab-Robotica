# Transformation Matrices - Scara_Final

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | Link_1 |
| $L_{2}$ | Cremallera2 |
| $L_{3}$ | Base_Link |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | Revoluci_n_2 | continuous (rad) | $L_{0}$ | $L_{1}$ |
| $q_{2}$ | Corredera_4 | prismatic (m) | $L_{0}$ | $L_{2}$ |
| $q_{3}$ | Revoluci_n_1 | continuous (rad) | $L_{1}$ | $L_{3}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  |-- [continuous] Revoluci_n_2 (q1)
  |   L1: Link_1
  |     +-- [continuous] Revoluci_n_1 (q3)
  |         L3: Base_Link
  +-- [prismatic] Corredera_4 (q2)
      L2: Cremallera2
```

## Transforms

## Revoluci_n_2

$L_{0}$ **base_link** -> $L_{1}$ **Link_1** (continuous)
  Variable: $q_{1}$

- **origin xyz**: (0, 0.0673, 0) m
- **origin rpy**: (1.570796, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{0}_{1}(q_{1}) = T_{fixed} \cdot R_{axis}(q_{1})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 0 & -1 & 0.0673 \\
0 & 1 & 0 & 0 \\
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

## Corredera_4

$L_{0}$ **base_link** -> $L_{2}$ **Cremallera2** (prismatic)
  Variable: $q_{2}$

- **origin xyz**: (0.0309, 0.291858, 0.0243) m
- **origin rpy**: (0, 0, 1.570796) rad
- **axis**: (1, 0, 0)
- **limits**: [0.015, 0.04] m

### Local Transform

$$
T^{0}_{2}(q_{2}) = \begin{bmatrix}
0 & -1 & 0 & 0.0309 + q_{2} \\
1 & 0 & 0 & 0.291858 \\
0 & 0 & 1 & 0.0243 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revoluci_n_1

$L_{1}$ **Link_1** -> $L_{3}$ **Base_Link** (continuous)
  Variable: $q_{3}$

- **origin xyz**: (-0.12, 0, 0.0738) m
- **origin rpy**: (3.141593, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{1}_{3}(q_{3}) = T_{fixed} \cdot R_{axis}(q_{3})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & -0.12 \\
0 & -1 & 0 & 0 \\
0 & 0 & -1 & 0.0738 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{3}) = \begin{bmatrix}
c_{3} & -s_{3} & 0 & 0 \\
s_{3} & c_{3} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

$$T^{0}_{3} = T^{0}_{1}(q_{1}) \cdot T^{1}_{3}(q_{3})\quad (L_0 \to L_{3}: \text{Base_Link})$$

