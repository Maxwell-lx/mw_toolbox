import numpy as np
from scipy.spatial import ConvexHull
from scipy import linalg
import matplotlib.pyplot as plt

'''
TODO：
1.
math_mbs.py:178: RuntimeWarning: invalid value encountered in divide
  r = np.arccos(n[2]) * r / np.linalg.norm(r)  # Euler rotation vector

'''


def exact_min_bound_sphere_3D(array):
    """
    Author: https://github.com/shrx/mbsc Date: Dec.2018
    Matlab code author: Anton Semechko (a.semechko@gmail.com) Date: Dec.2014
    REREFERENCES:
    [1] Welzl, E. (1991), 'Smallest enclosing disks (balls and ellipsoids)', Lecture Notes in Computer Science, Vol. 555, pp. 359-370
    [2] Ritter, J. (1990), 'An efficient bounding sphere', in Graphics Gems, A. Glassner, Ed. Academic Press, pp.301-303
    Compute exact minimum bounding sphere of a 3D point cloud (or a
    triangular surface mesh) using Welzl's algorithm.

   - X     : M-by-3 list of point co-ordinates or a triangular surface
             mesh specified as a TriRep object.
   - R     : radius of the sphere.
   - C     : 1-by-3 vector specifying the centroid of the sphere.
   - Xb    : subset of X, listing K-by-3 list of point coordinates from
             which R and C were computed. See function titled
"""
    # Get the convex hull of the point set
    hull = ConvexHull(array)
    hull_array = array[hull.vertices]
    hull_array = np.unique(hull_array, axis=0)
    # print('unique points=', len(hull_array))

    # Randomly permute the point set
    hull_array = np.random.permutation(hull_array)

    if len(hull_array) <= 4:
        R, C = _fit_sphere_2_points(hull_array)
        return R, C, hull_array

    elif len(hull_array) < 1000:
        try:
            R, C, _ = _B_min_sphere(hull_array, [])

            # Coordiantes of the points used to compute parameters of the
            # minimum bounding sphere
            D = np.sum(np.square(hull_array - C), axis=1)
            idx = np.argsort(D - R ** 2)
            D = D[idx]
            Xb = hull_array[idx[:5]]
            D = D[:5]
            Xb = Xb[D < 1E-6]
            idx = np.argsort(Xb[:, 0])
            Xb = Xb[idx]
            return R, C, Xb
        except:
            raise Exception
    else:
        M = len(hull_array)
        dM = min([M // 4, 300])
        # unnecessary ?
        #		res = M % dM
        #		n = np.ceil(M/dM)
        #		idx = dM * np.ones((1, n))
        #		if res > 0:
        #			idx[-1] = res
        #
        #		if res <= 0.25 * dM:
        #			idx[n-2] = idx[n-2] + idx[n-1]
        #			idx = idx[:-1]
        #			n -= 1

        hull_array = np.array_split(hull_array, dM)
        Xb = np.empty([0, 3])
        for i in range(len(hull_array)):
            R, C, Xi = _B_min_sphere(np.vstack([Xb, hull_array[i]]), [])

            # 40 points closest to the sphere
            D = np.abs(np.sqrt(np.sum((Xi - C) ** 2, axis=1)) - R)
            idx = np.argsort(D, axis=0)
            Xb = Xi[idx[:40]]

        D = np.sort(D, axis=0)[:4]
        Xb = np.take(Xb, np.where(D / R < 1e-3)[0], axis=0)
        Xb = np.sort(Xb, axis=0)

        return R, C, Xb


def _B_min_sphere(P, B):
    eps = 1E-6
    if len(B) == 4 or len(P) == 0:
        R, C = _fit_sphere_2_points(B)  # fit sphere to boundary points
        return R, C, P

    # Remove the last (i.e., end) point, p, from the list
    P_new = P[:-1].copy()
    p = P[-1].copy()

    # Check if p is on or inside the bounding sphere. If not, it must be
    # part of the new boundary.
    R, C, P_new = _B_min_sphere(P_new, B)
    if np.isnan(R) or np.isinf(R) or R < eps:
        chk = True
    else:
        chk = np.linalg.norm(p - C) > (R + eps)

    if chk:
        if len(B) == 0:
            B = np.array([p])
        else:
            B = np.array(np.insert(B, 0, p, axis=0))
        R, C, _ = _B_min_sphere(P_new, B)
        P = np.insert(P_new.copy(), 0, p, axis=0)
    return R, C, P


def _fit_sphere_2_points(array):
    """
    Fit a sphere to a set of 2, 3, or at most 4 points in 3D space. Note that
    point configurations with 3 collinear or 4 coplanar points do not have
    well-defined solutions (i.e., they lie on spheres with inf radius).

    - X     : M-by-3 array of point coordinates, where M<=4.
    - R     : radius of the sphere. R=Inf when the sphere is undefined, as
                specified above.
    - C     : 1-by-3 vector specifying the centroid of the sphere.
                C=nan(1,3) when the sphere is undefined, as specified above.
    """

    N = len(array)

    if N > 4:
        print('Input must a N-by-3 array of point coordinates, with N<=4')
        return

    # Empty set
    elif N == 0:
        R = np.nan
        C = np.full(3, np.nan)
        return R, C

    # A single point
    elif N == 1:
        R = 0.
        C = array[0]
        return R, C

    # Line segment
    elif N == 2:
        R = np.linalg.norm(array[1] - array[0]) / 2
        C = np.mean(array, axis=0)
        return R, C

    else:  # 3 or 4 points
        # Remove duplicate vertices, if there are any
        uniq, index = np.unique(array, axis=0, return_index=True)
        array_nd = uniq[index.argsort()]
        if not np.array_equal(array, array_nd):
            print("found duplicate")
            print(array_nd)
            R, C = _fit_sphere_2_points(array_nd)
            return R, C

        tol = 0.01  # collinearity/co-planarity threshold (in degrees)
        if N == 3:
            # Check for collinearity
            D12 = array[1] - array[0]
            D12 = D12 / np.linalg.norm(D12)
            D13 = array[2] - array[0]
            D13 = D13 / np.linalg.norm(D13)

            chk = np.clip(np.abs(np.dot(D12, D13)), 0., 1.)
            if np.arccos(chk) / np.pi * 180 < tol:
                R = np.inf
                C = np.full(3, np.nan)
                return R, C

            # Make plane formed by the points parallel with the xy-plane
            n = np.cross(D13, D12)
            n = n / np.linalg.norm(n)
            ##print("n", n)
            r = np.cross(n, np.array([0, 0, 1]))
            r = np.arccos(n[2]) * r / np.linalg.norm(r)  # Euler rotation vector
            ##print("r", r)
            Rmat = linalg.expm(np.array([
                [0., -r[2], r[1]],
                [r[2], 0., -r[0]],
                [-r[1], r[0], 0.]
            ]))
            ##print("Rmat", Rmat)
            # Xr = np.transpose(Rmat*np.transpose(array))
            Xr = np.transpose(np.dot(Rmat, np.transpose(array)))
            ##print("Xr", Xr)

            # Circle centroid
            x = Xr[:, :2]
            A = 2 * (x[1:] - np.full(2, x[0]))
            b = np.sum((np.square(x[1:]) - np.square(np.full(2, x[0]))), axis=1)
            C = np.transpose(_ldivide(A, b))

            # Circle radius
            R = np.sqrt(np.sum(np.square(x[0] - C)))

            # Rotate centroid back into the original frame of reference
            C = np.append(C, [np.mean(Xr[:, 2])], axis=0)
            C = np.transpose(np.dot(np.transpose(Rmat), C))
            return R, C

        # If we got to this point then we have 4 unique, though possibly co-linear
        # or co-planar points.
        else:
            # Check if the the points are co-linear
            D12 = array[1] - array[0]
            D12 = D12 / np.linalg.norm(D12)
            D13 = array[2] - array[0]
            D13 = D13 / np.linalg.norm(D13)
            D14 = array[3] - array[0]
            D14 = D14 / np.linalg.norm(D14)

            chk1 = np.clip(np.abs(np.dot(D12, D13)), 0., 1.)
            chk2 = np.clip(np.abs(np.dot(D12, D14)), 0., 1.)
            if np.arccos(chk1) / np.pi * 180 < tol or np.arccos(chk2) / np.pi * 180 < tol:
                R = np.inf
                C = np.full(3, np.nan)
                return R, C

            # Check if the the points are co-planar
            n1 = np.linalg.norm(np.cross(D12, D13))
            n2 = np.linalg.norm(np.cross(D12, D14))

            chk = np.clip(np.abs(np.dot(n1, n2)), 0., 1.)
            if np.arccos(chk) / np.pi * 180 < tol:
                R = np.inf
                C = np.full(3, np.nan)
                return R, C

            # Centroid of the sphere
            A = 2 * (array[1:] - np.full(len(array) - 1, array[0]))
            b = np.sum((np.square(array[1:]) - np.square(np.full(len(array) - 1, array[0]))), axis=1)
            C = np.transpose(_ldivide(A, b))

            # Radius of the sphere
            R = np.sqrt(np.sum(np.square(array[0] - C), axis=0))

            return R, C


def _permute_dims(array, dims):
    return np.transpose(np.expand_dims(array, axis=max(dims)), dims)


def _ldivide(array, vector):
    return np.linalg.solve(array, vector)


def fibonacci_sphere_sample(center, radius, N):
    # Check the inputs
    if len(center) != 3:
        raise ValueError("center must be a 3-dimensional vector")
    if radius <= 0:
        raise ValueError("radius must be a positive number")
    if N <= 0:
        raise ValueError("N must be a positive number")

    ga = (3 - np.sqrt(5)) * np.pi  # golden angle
    theta = ga * np.arange(N)  # golden angle increment
    z = 1 - 1.0 / N - (np.arange(N) / (N - 1)) * 2.0  # heights
    z = np.clip(z, -1, 1)  # limit z [-1,1]
    radius_at_z = np.sqrt(1 - z * z)  # radii
    x = radius_at_z * np.cos(theta)
    y = radius_at_z * np.sin(theta)
    points = np.stack([x, y, z]).T * radius
    points += center
    return points


def get_aabbs_bounding_sphere(aabbs):
    '''
    get bounding sphere of a list of aabbs
    1. extent aabb points from 2 to 8
    2. get the bounding sphere of those points
    :param aabbs: aabb get from pybullet.getAABB(robot.id, i) , i-th link of robot, shape [2,3],aabbs shape [n,2,3]
    :return: center, radius
    '''
    points = []
    for i in range(len(aabbs)):
        a = aabbs[i][0]
        b = aabbs[i][1]
        c = (a[0], b[1], a[2])
        d = (a[0], b[1], b[2])
        e = (b[0], b[1], a[2])
        f = (b[0], a[1], a[2])
        g = (b[0], a[1], b[2])
        h = (a[0], a[1], b[2])
        points += [a, b, c, d, e, f, g, h]

    points = np.array(points)
    radius, center, _ = exact_min_bound_sphere_3D(points)
    return center, radius


def homo2cart(homo_co):
    '''
    transfer homogeneous coordinate(s) to cartesian coordinate(s)
    :param homo_co: list, shape [3,1] or [3,n]
    :return:
    '''
    if len(homo_co.shape) > 1:
        cart_co = homo_co[:-1, :] / homo_co[-1, :]
    else:
        cart_co = homo_co[:-1] / homo_co[-1]
    return cart_co


if __name__ == '__main__':
    # fibonacci_sphere_sample
    center = [1, 2, 3]
    radius = 4
    N = 1000

    points = fibonacci_sphere_sample(center, radius, N)

    # plot the points
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2])
    plt.show()

    # test exact_min_bound_sphere_3D
    print('P_num < 4')
    P = np.random.rand(4, 3)
    print('P:\n', P)
    R, C = _fit_sphere_2_points(P)
    print('R:', R, 'C:', C, sep="\n")

    print('P_num >= 4')
    points = np.array([
        [1, 2, 3],
        [4, 2, 2],
        [7, 7, 2],
        [10, 11, 3],
        [13, 2, 8],
        [16, 17, 18]])

    # np.random.shuffle(points)
    print(points)
    radius, center, xb = exact_min_bound_sphere_3D(points)

    print("Bounding sphere center:", center)
    print("Bounding sphere radius:", radius)
    print(xb)
