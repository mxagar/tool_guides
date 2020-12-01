#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

int main(int argc, char** argv)
{
    // --- BASICS: Matrices & Vectors
    std::cout << "\n--- Basics: Matrices & Vectors ---" << std::endl;

    // MatrixXd @ Eigen/Dense: any size X (dynamic allocation / size determined at run time), double type d
    MatrixXd m(2, 2); // size 2x2 defined here
    m(0, 0) = 3; // access with (), row is first index, column next
    m(1, 0) = 2.5;
    m(0, 1) = -1;
    m(1, 1) = m(1, 0) + m(0, 1);
    std::cout << "m = \n" <<  m << std::endl; // [[3, -1],[2.5, 1.5]]
    std::cout << "m(0, 1, 2, 3) = \n" << m(0) << ", " << m(1) << ", " << m(2) << ", " << m(3) << std::endl; // COLUMN-MAJOR STORAGE: 3, 2.5, -1, 1.5
    Matrix3f mm; // X = 3, float <- size specified at compile time
    mm << 1, 2, 3,
        4, 5, 6,
        7, 8, 9; // Comma-initialization (for known-size matrices): ROW-MAJOR!!
    std::cout << "mm = \n" << mm << std::endl; // [[1,2,3],[4,5,6],[7,8,9]]
    MatrixXd mr = MatrixXd::Random(3,3); // any random value [-1,1]
    MatrixXd mc = MatrixXd::Constant(3,3,1.2); // 3x3, value 1.2 for all
    mr = mr + mc;
    std::cout << "mr = \n" << mr << std::endl;

    // VectorXd @ Eigen/Dense: any size X (dynamic allocation), double type d
    // Vectors are column (default) or row matrices
    VectorXd v(3);
    v << 1, 2, 3; // comma-initialization (also for unknown-size vectors) -> [1, 2, 3]^T
    std::cout << "v(1) = " << v(1) << std::endl;
    
    // Vector x Maxtrix
    VectorXd mrv = mr * v;
    std::cout << "mr * v = \n" << mrv << std::endl;

    // --- MATRIX AND VECTOR OPERATIONS
    std::cout << "\n--- Matrix and Vector Operations ---" << std::endl;

    // All matrices and vectors are typedefs of the template class Eigen::Matrix:
    // Matrix<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
    //      Scalar: int, float, double, ...
    //      RowsAtCompileTime, ColsAtCompileTime: a concrete value 1, 2, 3... or Dynamic (if size unknown at compile time)
    // Examples of typedefs in Eigen:
    //      typedef Matrix<float, 4, 4> Matrix4f;
    //      typedef Matrix<float, 3, 1> Vector3f;
    //      typedef Matrix<int, Dynamic, 1> VectorXi;
    //      typedef Matrix<int, 1, 2> RowVector2i;
    //      In general
    //      - MatrixNt for Matrix<type, N, N>
    //      - VectorNt for Matrix<type, N, 1>
    //      - RowVectorNt for Matrix<type, 1, N>
    //      where
    //          N = 2, 3, 4, X
    //          type = i (int), f (float), d (double), cf (complex float), cd (complex double)
    // Default constructors always available, only memory allocation performed (if size known)
    //      Matrix3f a; // float[9] allocated
    //      MatrixXf b; // dynamic size 0-by-0
    //      MatrixXf a(10,15); // dynamic size 10-by-15
    // Value initialization performed only for Vectors, if values passed
    //      Vector3d b(5.0, 6.0, 7.0); // [5.0, 6.0, 7.0]^T
    // Fixed vs Dynamic sizes
    //      if size < 4x4, try to use fixed size, if possible
    //      otherwise, dynamic size; note:
    //      - dynamic objects are stored in heap
    //      - for large sizes it's better to have dynamic allocation

    // Resizing: we can resize matrices (caveat: data might be erased)
    std::cout << "m rows & columns = " << m.rows() << ", " << m.cols() << std::endl;
    m.resize(4,3);
    std::cout << "m rows & columns = " << m.rows() << ", " << m.cols() << std::endl;
    std::cout << "m size = " << m.size() << std::endl;

    // Assignment: we can assign matrices of different sizes, they are resized
    MatrixXf a(2,2);
    MatrixXf b(3,3);
    std::cout << "a rows & columns = " << a.rows() << ", " << a.cols() << std::endl; // 2, 2
    a = b; // a becomes 3x3
    std::cout << "a rows & columns = " << a.rows() << ", " << a.cols() << std::endl; // 3, 3

    // Arithmetics
    a.resize(2,2);
    b.resize(2,2);
    Vector2f u(9, 10);
    a << 1, 2,
        3, 4;
    b << 5, 6,
        7, 8;
    Matrix2f c;

    c = a + b;
    c = a - b;
    c = -a;
    c += a;
    c -= b;

    u = 1.5*u;
    c = 2.5*c;

    c = a.transpose();
    c = a.conjugate(); // no operation is a is real
    c = a.adjoint(); // transpose if a is real
    // NOTE: a = a.transpose() DOES NOT WORK!

    Vector2f w = c*u; // Matrix x Vector
    c = a*b; // Matrix x Matrix

    Vector3f e1(1,0,0), e2(0,1,0);
    Vector3f e3 = e1.cross(e2);
    std::cout << "e3 = e1 x e2 = \n" << e3 << std::endl;
    float e1e2 = e1.dot(e2);
    std::cout << "e1.e2 = " << e1e2 << std::endl;

    Matrix2d mat;
    mat << 1, 2,
            3, 4;
    std::cout << "mat.sum():       " << mat.sum()       << std::endl;
    std::cout << "mat.prod():      " << mat.prod()      << std::endl;
    std::cout << "mat.mean():      " << mat.mean()      << std::endl;
    std::cout << "mat.minCoeff():  " << mat.minCoeff()  << std::endl;
    std::cout << "mat.maxCoeff():  " << mat.maxCoeff()  << std::endl;
    std::cout << "mat.trace():     " << mat.trace()     << std::endl;
    std::cout << "mat.trace():     " << mat.diagonal().trace()     << std::endl;

    // --- ARRAYS
    std::cout << "\n--- Arrays: Coefficient/Element-wise operations ---" << std::endl;

    // Matrix class is for linear algebra
    // Array class is general purpose container (not for linear algebra) and has element/coefficient-wise operations
    // When to use which?
    // -> for linear algebra operations: Matrix
    // -> for element-wise operations: Array
    // As for Matrix, typedefs are defined (note: 2-dimensional arrays have double dimension N)
    //      Array<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
    //      Array<float,Dynamic,1> -> ArrayXf 
    //      Array<float,3,1> -> Array3f
    //      Array<double,Dynamic,Dynamic> -> ArrayXXd
    //      Array<double,3,3> -> Array33d 

    ArrayXXf x(3,3);
    ArrayXXf y(3,3);
    x << 1,2,3,
        4,5,6,
        7,8,9;
    y << 1,2,3,
        1,2,3,
        1,2,3;

    // Adding two arrays (element-wise)
    std::cout << "x + y = \n" << std::endl << x + y << std::endl;
    // Subtracting a scalar from an array (element-wise)
    std::cout << "x - 2 = \n" << std::endl << x - 2 << std::endl;
    // Multiplying (element-wise)
    std::cout << "x * y = \n" << x * y << std::endl;

    // More operations
    ArrayXf r = ArrayXf::Random(5);
    r *= 2;
    std::cout << "r =" << std::endl 
        << r << std::endl;
    std::cout << "r.abs() =" << std::endl 
        << r.abs() << std::endl;
    std::cout << "r.abs().sqrt() =" << std::endl 
        << r.abs().sqrt() << std::endl;
    std::cout << "r.min(r.abs().sqrt()) =" << std::endl 
        << r.min(r.abs().sqrt()) << std::endl;

    // Conversion Matrix <-> Array: .matrix(), .array()
    MatrixXf A(2,2);
    MatrixXf B(2,2);
    MatrixXf R(2,2);
 
    A << 1,2,
        3,4;
    B << 5,6,
        7,8;

    R = A * B;
    std::cout << "Matrix A*B:" << std::endl << R << std::endl << std::endl;
    R = A.array() * B.array();
    std::cout << "Array A*B:" << std::endl << R << std::endl << std::endl;
    R = A.cwiseProduct(B);
    std::cout << "With cwiseProduct (constant-wise product):" << std::endl << R << std::endl << std::endl;
    R = A.array() + 4;
    std::cout << "Array A + 4:" << std::endl << R << std::endl << std::endl;

    R = (A.array() + 4).matrix() * A;
    std::cout << "Combination 1:" << std::endl << R << std::endl << std::endl;
    R = (A.array() * B.array()).matrix() * A;
    std::cout << "Combination 2:" << std::endl << R << std::endl << std::endl;

    // --- LINEAR ALGEBRA EQUATIONS AND DECOMPOSITIONS
    std::cout << "\n--- Linear Algebra Equations and Decompositions ---" << std::endl;

    // Example: Ax = b with Housholder QR
    Matrix3f AA;
    Vector3f bb;
    AA << 1,2,3,  4,5,6,  7,8,10;
    bb << 3, 3, 4;
    std::cout << "Ax = b with Housholder QR:" << std::endl;
    std::cout << "Here is the matrix A:\n" << AA << std::endl;
    std::cout << "Here is the vector b:\n" << bb << std::endl;
    Vector3f xx = AA.colPivHouseholderQr().solve(bb);
    std::cout << "The solution is:\n" << xx << std::endl;

    // colPivHouseholderQr() returns an object of the type ColPivHouseholderQR, which is used to solve the linear system
    // ColPivHouseholderQR performs a QR decomposition with column pivoting
    // Other useful decompositions that can be used for solving linear systems (see links, there are more classes):
    // https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
    // https://eigen.tuxfamily.org/dox/group__TopicLinearAlgebraDecompositions.html
    // https://eigen.tuxfamily.org/dox/group__DenseDecompositionBenchmark.html
    // - PartialPivLU
    // - FullPivLU
    // - HouseholderQR
    // - ColPivHouseholderQR
    // - FullPivHouseholderQR
    // - LLT
    // - BDCSVD

    // Example: AX=B with Cholesky
    A << 2, -1, -1, 3;
    B << 1, 2, 3, 1;
    std::cout << "AX=B with Cholesky:" << std::endl;
    std::cout << "Here is the matrix A:\n" << A << std::endl;
    std::cout << "Here is the right hand side b:\n" << B << std::endl;
    Matrix2f X = A.ldlt().solve(B);
    std::cout << "The solution is:\n" << X << std::endl;

    // Check if a solution exists: Just compute the error and decide if we're OK with it
    MatrixXd AAA = MatrixXd::Random(100,100);
    MatrixXd bbb = MatrixXd::Random(100,50);
    MatrixXd xxx = AAA.fullPivLu().solve(bbb);
    double relative_error = (AAA*xxx - bbb).norm() / bbb.norm(); // norm() is L2 norm
    std::cout << "The relative error is:\n" << relative_error << std::endl;

    // Eigenvalues an Eigenvectors
    A << 1, 2, 2, 3;
    std::cout << "Eigenvalues and Eigenvectors:" << std::endl;
    std::cout << "A:\n" << A << std::endl;
    SelfAdjointEigenSolver<Matrix2f> eigensolver(A); // There are more options/eigen-solvers
    if (eigensolver.info() != Success) abort();
    std::cout << "eigenvalues(A):\n" << eigensolver.eigenvalues() << std::endl;
    std::cout << "eigenvectors(A):\n"
        << eigensolver.eigenvectors() << std::endl;

    // Inverse and Determinant
    // Note: make sure you really want the inverse; usually, it's successfully replaced by .solve()
    // Some decomposition classes (see above) offer .inverse() and .determinant() methods
    // For small matrix sizes (e.g., 4x4), custom more efficient methods are implemented
    std::cout << "Inverse and Determinant:" << std::endl;
    AA << 1, 2, 1,
        2, 1, 0,
        -1, 1, 2;
    std::cout << "A = \n" << AA << std::endl;
    std::cout << "det(A) = \n" << AA.determinant() << std::endl;
    std::cout << "inv(A) = \n" << AA.inverse() << std::endl;

    // --- LEAST SQUARES
    std::cout << "\n--- Least Squares ---" << std::endl;
    // Best solved with SVD decompositions; 2 classes:
    // 1. BDCSVD: for large problems
    // 2. JacobiSVD: for smaller problems
    // See also:
    // https://eigen.tuxfamily.org/dox/group__LeastSquares.html

    MatrixXf M = MatrixXf::Random(3, 2);
    std::cout << "M = \n" << M << std::endl;
    VectorXf z = VectorXf::Random(3);
    std::cout << "z = \n" << z << std::endl;
    std::cout << "M*q = z -> q = \n" << M.bdcSvd(ComputeThinU | ComputeThinV).solve(z) << std::endl;

    // --- GEOMETRY: SPATIAL TRANSFORMATIONS
    std::cout << "\n--- Geometry: Spatial Transformations ---" << std::endl;

    // 2D & 3D rotations, projective and affine transformations
    // Two types of transformation objects:
    // 1. Abstract object, not represented by matrices, but which can operate with matrices
    //      rotations: Eigen::AngleAxis, Eigen::Quaternion
    //          NOTE: we can create a rotation object type from another type, but it must be in 2 steps, not in the constructor, eg
    //              INCORRECT: Quaternion<float> q = AngleAxis<float>(angle_in_radian, axis);
    //              CORRECT: Quaternion<float> q; q = AngleAxis<float>(angle_in_radian, axis);
    //      translations: Eigen::Translation
    //      scalings: Eigen::Scaling
    // 2. Transformations, represented by matrices: Projective and Affine Transformations
    //      Eigen::Transform
    // Note: if matrices, recall data is stored in column-major order

    // Rotation
    float angle_in_radian = 1.0;
    Rotation2D<float> rot2(angle_in_radian); // 2D rotation
    float ax = 1.0, ay = 0.0, az = 0.0;
    Vector3f axis(ax,ay,az);
    AngleAxis<float> aa(angle_in_radian, axis);
    Quaternion<float> q; q = AngleAxis<float>(angle_in_radian, axis);
    
    // Scaling
    float sx = 1.0f, sy = 1.2f, sz = 1.3f;
    Eigen::AlignedScaling2f S1= Scaling(sx, sy);
    Eigen::AlignedScaling3f S2 = Scaling(sx, sy, sz);
    Vector3f sN(sx, sy, sz);
    Eigen::AlignedScaling3f S3 = Scaling(sN);

    // Translation
    float tx = 0.0, ty = 1.0, tz = -1.0;
    Translation<float,2> t1(tx, ty);
    Translation<float,3> t2(tx, ty, tz);
    Translation<float,3> t3(Vector3f(tx, ty, tz));

    // Affine transformations (Transform class): Concatenation
    Transform<float,3,Affine> H1 = Translation3f(Vector3f(tx, ty, tz)) * AngleAxisf(angle_in_radian,axis) * Scaling(sN);
    std::cout << "rxx: H1(0,0) = " << H1(0,0)
                << ", ryy: H1(1,1) = " << H1(1,1)
                << ", rzz: H1(2,2) = " << H1(2,2)
                << ", tx: H1(0,3) = " << H1(0,3)
                << ", ty: H1(1,3) = " << H1(1,3)
                << ", tz: H1(2,3) = " << H1(2,3)
                << ", 1: H1(3,3) = " << H1(3,3)
                << ", 0: H1(3,2) = " << H1(3,2)
                << std::endl;

    // Linear transformations (Matrix class): Concatenation
    Matrix2f H2 = Rotation2Df(angle_in_radian) * Scaling(Vector2f(sx,sy));
    Matrix3f H3 = AngleAxisf(angle_in_radian,axis) * Scaling(Vector3f(sx,sy,sz));
    H1 = H3; // An affine transform can accept a linear transformation as input

    // Typical transformations: Homogeneous matrices (rotation and translation), pose composition, point transformation
    // Instead of Matrix3f, Affine3f seems better suited for homogeneous transformations
    // because it accepts during constructions other object types, such as AngleAxisf or Translation3f
    // However, note that when done that, the assignation must be carried out in two steps, not in constructor, eg
    // INCORRECT (compilation error): Affine3f Rot = AngleAxisf(rad,Vector3f(1.0, 0.0, 0.0));
    // CORRECT: Affine3f Rot; Rot = AngleAxisf(rad,Vector3f(1.0, 0.0, 0.0));
    // NOTE: Affine3f is typedef Transform<float,3,Affine>
    Vector3f P1(1.0, 2.0, 3.0);
    float rad = 0.5f;
    Affine3f I = Affine3f::Identity();
    Affine3f Rot; Rot = AngleAxisf(rad,Vector3f(1.0f, 0.0f, 0.0f));
    Affine3f T; T = Translation3f(Vector3f(0.5f, -0.6f, 0.7f));
    Affine3f H4 = T * Rot; // [Rot|T]
    Affine3f H5 = I * H1.inverse() * H4; // Concatenate transformations
    Vector3f P2 = H5 * P1; // Transform point
    std::cout << "P2 = " << P2(0) << ", " << P2(1) << ", " << P2(2) << std::endl;

    // Get components
    Matrix4f MH5 = H5.matrix();
    Vector3f TH5 = H5.translation();
    Matrix3f RH5 = H5.rotation();

    // Euler Angles
    // There are 24 different conventions
    // Example: Rotation matrix according to the 2-1-2 (Z-Y-Z) convention (0:x, 1:y, 2:z):
    Matrix3f EulerRot;
    float angle1 = 0.5f, angle2 = 0.7f, angle3 = 0.9f;
    EulerRot = AngleAxisf(angle1, Vector3f::UnitZ()) * AngleAxisf(angle2, Vector3f::UnitY()) * AngleAxisf(angle3, Vector3f::UnitZ());
    Vector3f Angles = EulerRot.eulerAngles(2, 1, 2); // Z-Y-Z
    std::cout << "Euler angles = " << Angles(0) << ", " << Angles(1) << ", " << Angles(2) << std::endl;
    
}