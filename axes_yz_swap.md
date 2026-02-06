# Y/Z Axis Swap (Quest → Foxglove)

## Why this is needed
Quest pose data is logged in a coordinate basis that does not match Foxglove's
default visualization convention. Foxglove uses a right-handed frame where:

- X = right
- Y = forward (away from the viewer by default)
- Z = up

The Quest data we are using effectively has Y and Z swapped relative to that
convention. To visualize correctly in Foxglove, we convert from the Quest frame
into the Foxglove frame by swapping Y and Z.

## The basis change matrix
A basis swap is a linear transform. For a Y/Z swap:

```
M = [ 1 0 0
      0 0 1
      0 1 0 ]
```

Apply it to position:

```
p_fox = M * p_quest
```

This yields:

```
(x, y, z) -> (x, z, y)
```

## Rotations must also be transformed
Rotations are not vectors, so they must be transformed using the full rotation
matrix. Convert the Quest quaternion to a rotation matrix R_quest, then apply:

```
R_fox = M * R_quest * Mᵀ
```

Finally, convert R_fox back to a quaternion for publishing.

This keeps the rotation physically consistent with the swapped axes.

## Why not just swap quaternion components?
Directly swapping quaternion components only works for a pure axis permutation
with no sign flips and no handedness change. The matrix method is safe for all
cases (including future sign flips).

## Summary
1. Convert quaternion → rotation matrix
2. Apply R_fox = M * R_quest * Mᵀ
3. Convert back to quaternion
4. Apply p_fox = M * p_quest
