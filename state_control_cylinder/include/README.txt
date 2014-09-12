A few notes on the code for computing the formation control.

# TYPES FOR VECTORS AND MATRICES
I use the TooN library (http://www.edwardrosten.com/cvd/toon.html) for
matrix/vector computations. The header vectorTypes.hpp contains
typedefs for vectors in R2, in R3 and 2x2 matrices and wrapper
functions for some common operations. The types with an extra V in
front  (e.g., VVec3) are typedefs of std::vectors of that type (e.g.,
std::vector<Vec3>).
If there is the need to port the code to another linear algebra
library, one (in theory) needs only to change this header.

# NOMENCLATURE
In general, the word "bearings" refers to normalized vectors, and the
word "ranges" refers to the magnitudes of the vectors (which are
scalars). Arguments called "y" and "yg" refer to current bearings and
desired bearings (i.e., the bearings that specify the formation).
Arguments called "ny" and "nyg" refer to the corresponding range
information. All the vectors (except for y0 in controlYaw() ), are
assumed to be in the world's reference frame.

# OVERVIEW OF THE FUNCTIONS
There are three main functions.
## controlBearing
Takes as argument the current and desired bearings and
returns the velocity control vector corresponding to the bearing only
part of the cost (\phi^b in the paper).
## controlRange
Takes as argument the current and desired bearings and ranges. Returns
the velocity control vector corresponding to the bearing+range part of
the cost (\phi^r in the paper).
## controlYaw
Takes as argument the current bearings, the current yaw angle theta,
and a 2-D vector y0 in the body's reference frame denoting the optical
axis (i.e., the center of the field of view, typically the y axis [0;1]).
Returns the velocity control for the yaw angle, aiming to bring the
all the bearings withing a safe angle around the optical axis.
The field-of-view and safe-field-of-view angles are specified (in
degrees) as macros at the top of bearingNetwork.hpp
The angle theta is internally converted to a 2-D rotation
R=[cos(theta), -sin(theta);
     sin(theta), cos(theta)]
representing the world-to-body change of coordinates.
The control will return values of "inf" if any of the bearings is
outside the field of view.

# USING THE FUNCTIONS
The functions controlBearing and controlRange use complementary
information (see the paper), and their results should be combined. For
instance, if dxBearing is the output of controlBearing and dxRange is
the output of conrolRange, the final velocity control should be a
linear combination of the kind dx=alpha1*dxBearing+alpha2*dxRange.
Note also that, for now, the yaw control simply aims to bring all the bearings in the "safe" field
of view, and it does not consider the formation control
objective. This control is also essentially 2-D. If you pass 3-D
bearings, they are first converted to 2-D bearings by projecting onto
the xy-plane and normalizing.

# FILES
## bearingNetworkTest.cpp
Runs some tests against results
obtained with the Matlab implementation, and gives examples of how to
call the functions.
## bearingNetwork.*
Implementation of the control laws.
## vectorTypes.hpp
Header with typedefs for matrices/vectors and common vector operations.
## vectorUtilities.hpp
Some handy functions for working on std::vector's
## testUtilities.*
Some handy functions for performing the tests in bearingNetworkTest.cpp
