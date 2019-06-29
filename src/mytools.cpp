#include "mytools.h"
#include <Eigen/SVD>
#include <random>
#include <Eigen/Dense>

#define PI 3.14159265

void get_axis(Eigen::MatrixXd & axis)
{
    axis = (Eigen::MatrixXd (31,3) <<
            0   ,0  ,0,
            0.01,0  ,0,
            0.02,0  ,0,
            0.03,0  ,0,
            0.04,0  ,0,
            0.05,0  ,0,
            0.06,0  ,0,
            0.07,0  ,0,
            0.08,0  ,0,
            0.09,0  ,0,
            0.1 ,0  ,0,
            0  ,0.01,0,
            0  ,0.02,0,
            0  ,0.03,0,
            0  ,0.04,0,
            0  ,0.05,0,
            0  ,0.06,0,
            0  ,0.07,0,
            0  ,0.08,0,
            0  ,0.09,0,
            0  ,0.1 ,0,
            0  ,0   ,0.01,
            0  ,0   ,0.02,
            0  ,0   ,0.03,
            0  ,0   ,0.04,
            0  ,0   ,0.05,
            0  ,0   ,0.06,
            0  ,0   ,0.07,
            0  ,0   ,0.08,
            0  ,0   ,0.09,
            0  ,0   ,0.1  ).finished() ;
}


void calculate_vertex_normal(Eigen::MatrixXd const & V, Eigen::MatrixXi const & F, Eigen::MatrixXd const & FN, Eigen::MatrixXd & out_VN)
{
    //
    // input:
    //   V: vertices
    //   F: face
    //   FN: face normals
    // output:
    //   out_VN
    //
    //   Your job is to implement vertex normal calculation
    //
    
    out_VN.resize(V.rows(), V.cols());
    out_VN.setZero();
    
    std::vector<std::vector<int> > VF; //#V list of lists of incident faces (adjacency list)
    std::vector<std::vector<int> > VFi;//#V list of lists of index of incidence within incident faces listed
    igl::vertex_triangle_adjacency(V.rows(), F, VF, VFi);
    
    //std::cout << "Face Normals " << FN << std::endl;
    
    // loop for each vertex
    for (int vert = 0; vert<V.rows(); vert++) {
        //std::cout << "Vertex " << vert << std::endl;
        
        // accumulator of the normals for the output
        Eigen::MatrixXd accumulator;
        accumulator.resize(1, FN.cols());
        accumulator.setZero();
        
        // loop for the neighbor faces of current vertex
        for (int neighbor = 0; neighbor < VF[vert].size(); neighbor++) {
            accumulator = accumulator + FN.row(VF[vert][neighbor]);
        }
        accumulator.normalize(); // normalize the normal vector of the vertex
        
        // output
        out_VN.row(vert) = accumulator;
    }
    
}

void calculate_vertex_normal_flann(Eigen::MatrixXd const & V, Eigen::MatrixXi const & F, Eigen::MatrixXd & out_VN)
{
    //
    // input:
    //   V: vertices
    //   F: face
    //   FN: face normals
    // output:
    //   out_VN
    //
    // Your job is to implement vertex normal calculation vis using flann and igl:fitplane
    //
    // igl::fit_plane(V, N, C);
    // Input:
    //   V #Vx3 matrix. The 3D point cloud, one row for each vertex.
    // Output:
    //   N 1x3 Vector. The normal of the fitted plane.
    //   C 1x3 Vector. A point that lies in the fitted plane.
    //
    
    out_VN.resize(V.rows(), V.cols());
    out_VN.setZero();

    // build tree
    nanoflann::KDTreeEigenMatrixAdaptor< Eigen::MatrixXd > mat_index( V, 50); // 50 is the max leaf
    mat_index.index->buildIndex();

    Eigen::RowVector3d v_cen = V.colwise().sum() / V.rows();
    
    // for each vertex
    for (int vert = 0; vert<V.rows(); vert++) {
        Eigen::RowVector3d rd_pts = V.row(vert);
        
        // set K nearest samples
        const size_t par_K = 10;
        
        // create a query object
        std::vector<size_t> indexes(par_K);
        std::vector<double> dists_sqr(par_K);
        
        nanoflann::KNNResultSet<double> res(par_K);
        res.init(indexes.data(), dists_sqr.data());
        
        // find KNN
        mat_index.index->findNeighbors(res, rd_pts.data(), nanoflann::SearchParams(50));
        
        Eigen::MatrixXd nn_vex(indexes.size(),3);
        // loop for all nearest neighbors
        for (size_t i=0 ; i < indexes.size() ;i++){
            nn_vex.row(i) = V.row(indexes[i]);
        }
        // fit a plane on the nearest neighbors vertices and find the normal of the plane
        Eigen::RowVector3d N;
        Eigen::RowVector3d C;
        igl::fit_plane(nn_vex,N,C);

        if(vert>0){
            if(N.dot(v_cen - rd_pts)>0){
                N = -N;
            }
        }
        //output
        out_VN.row(vert) = N;
    }
}


void roughlyAlign(Eigen::MatrixXd & m2_V, double xAngle, double yAngle,double zAngle,double xT,double yT,double zT){
    
    xAngle = xAngle * PI/180;
    yAngle = yAngle * PI/180;
    zAngle = zAngle * PI/180;
    // roughly align the 2 could points through rigid transformation

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    /*
    T<< cos(xAngle)*cos(yAngle), cos(xAngle)*sin(yAngle)*sin(zAngle)-sin(xAngle)*cos(zAngle), cos(xAngle)*sin(yAngle)*cos(zAngle)+sin(xAngle)*sin(zAngle), xT,
    sin(xAngle)*cos(yAngle), sin(xAngle)*sin(yAngle)*sin(zAngle)+cos(xAngle)*cos(zAngle), sin(xAngle)*sin(yAngle)*cos(zAngle)-cos(xAngle)*sin(zAngle), yT,
    -sin(yAngle)           , cos(yAngle)*sin(zAngle)                                    , cos(yAngle)*cos(zAngle) , zT,
    0                      , 0                                                          , 0                       , 1;
    */
    T.topLeftCorner(3, 3) = getRotationMatrix(xAngle,yAngle,zAngle);
    T.topRightCorner(3, 1) << xT, yT, zT;
    T.bottomLeftCorner(1, 3) << 0,0,0;
    std::cout << "T " << T << std::endl;

    // transform in homogeneous coordinates
    m2_V.conservativeResize(m2_V.rows(), 4);
    m2_V.col(3).setOnes();
    
    // rigid transform
    Eigen::MatrixXd V2 =(T * (m2_V.transpose()) ).transpose();

    //get rid of the 4th col
    Eigen::MatrixXd V3 = V2.leftCols(3);
    // divide by the 4th col
    V3 = V3.array().colwise() / V2.array().col(3);

    m2_V = V3;

}


Eigen::MatrixXd findNN(Eigen::MatrixXd Q, Eigen::MatrixXd P, size_t numberOfNN){
    Eigen::MatrixXd NNs = Eigen::MatrixXd::Zero(Q.rows(), Q.cols());
    // build tree
    nanoflann::KDTreeEigenMatrixAdaptor< Eigen::MatrixXd > mat_index( P, 100); // 50 is the max leaf
    mat_index.index->buildIndex();
    
    // iterate for every q in Q to find its nearest neighbor
    for (int idx = 0; idx < Q.rows(); idx++) {
        // current q
        Eigen::RowVector3d q_i = Q.row(idx);
        
        // set the parameters, how many neighbors to find (num_results = 2 means : finds 1 nearest neighbor, + itself)
        const size_t num_results = numberOfNN;
        std::vector<size_t>   ret_indexes(num_results);
        std::vector<double> out_dists_sqr(num_results);
        
        // resultSet will contain the nearest neighbors
        nanoflann::KNNResultSet<double> resultSet(num_results);
        
        // find nearest neighbors
        resultSet.init(ret_indexes.data(), out_dists_sqr.data());
        mat_index.index->findNeighbors(resultSet, q_i.data() , nanoflann::SearchParams(25));
        
        
        NNs.row(idx) = P.row(ret_indexes[0]);
    }
    
    return NNs;
}


void ICP_p2Point(Eigen::MatrixXd & Q, Eigen::MatrixXd const P){
    
    // Q is the mesh to move
    // P is the fixed mesh ( destination )

    std::cout<< "Point to point " << std::endl;

    // for convergence condition
    double error = 10000;
    int iter = 0;
    double threshold = 0.000001;
    double previous_error = 100000;

    std::cout << "threshold " << threshold <<  std::endl;


    while ( error > threshold && iter < 50 && abs(previous_error-error) > threshold*0.01) {
        Eigen::MatrixXd source = Q;

        // matrix of nearest neighbors
        size_t numberOfNN = 1;
        Eigen::MatrixXd NNs = findNN(Q, P, numberOfNN);
        
        Eigen::MatrixXd destination = NNs;
        
        //rejectPoints(Q,NNs);
        
        //centroids
        Eigen::RowVector3d P_cen = NNs.colwise().sum() / NNs.rows();
        Eigen::RowVector3d Q_cen = Q.colwise().sum() / Q.rows();

        // center point clouds
        Eigen::MatrixXd Q_hat;
        Q_hat = Q.rowwise() - Q_cen;
        Eigen::MatrixXd P_hat;
        P_hat = NNs.rowwise() - P_cen;
        
        // find R and t
        Eigen::Matrix3d A(3,3);
        A = Q_hat.transpose() * P_hat; // 3xN * Nx3
        //std::cout << " A "<< A << std::endl;
        // SVD decomposition
        Eigen::JacobiSVD<Eigen::MatrixXd> A_svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d U = A_svd.matrixU();
        Eigen::Matrix3d V = A_svd.matrixV();
        
        // calculate the rotation and translation
        Eigen::Matrix3d ifNeg = Eigen::Matrix3d::Identity();
        ifNeg(2,2) = (V*(U.transpose())).determinant();
        Eigen::Matrix3d R = V*ifNeg*(U.transpose());
        Eigen::MatrixXd t(3,1);
        t = P_cen.transpose() - R*Q_cen.transpose();

        // homogeneous transformation
        Eigen::Matrix4d T = Eigen::MatrixXd::Zero(4, 4);
        T.topLeftCorner(3, 3) = R;
        T.topRightCorner(3, 1) = t;
        T(3,3) = 1;

        // transform source points in homogeneous coordinates
        Eigen::MatrixXd Q_hom = Eigen::MatrixXd::Ones(source.rows(),source.cols()+1);//source
        Q_hom.leftCols(3) = source;//Q;// source
        
        // apply rigid transformation to source
        Eigen::MatrixXd movedPoints = (T*Q_hom.transpose()).transpose();
        movedPoints = movedPoints.array().colwise() / movedPoints.array().col(3);
        movedPoints = movedPoints.leftCols(3);
        
        // point to point measure
        previous_error = error;
        error = ((destination.array() - movedPoints.array()).pow(2)).sum() / movedPoints.rows();

        std::cout <<"iter " << iter <<" error " << error << ", error difference " << abs(previous_error-error) <<  std::endl;


        Q = movedPoints;
        iter += 1;
    }
    
}

void rejectPoints(Eigen::MatrixXd & Q, Eigen::MatrixXd & NNs){

    Eigen::MatrixXd Q_  = Eigen::MatrixXd::Zero(Q.rows(), 3);
    Eigen::MatrixXd NNs_= Eigen::MatrixXd::Zero(Q.rows(), 3);

    Eigen::MatrixXd NNs_Normals;
    Eigen::MatrixXd Q_normals;
    Eigen::MatrixXi F;
    // find normals of Q and NNs
    calculate_vertex_normal_flann(Q, F, Q_normals);
    calculate_vertex_normal_flann(NNs, F, NNs_Normals);

    int j = -1;
    float normalsAngle;
    for (int i=0; i<Q.rows(); i++) {
        // cosine between normals of Q and NNs
        normalsAngle = abs(Q_normals.row(i).dot(NNs_Normals.row(i)));
        
        if (normalsAngle > 0.975) { // compare with a threshold
            j += 1;
            Q_.row(j) = Q.row(i);
            NNs_.row(j) = NNs.row(i);
        }
    }

    Q_.conservativeResize(j, 3);
    NNs_.conservativeResize(j, 3);
    std::cout << "Q points before" << Q.rows() << std::endl;
    Q = Q_;
    std::cout << "Q points after " << Q.rows() << std::endl;
    NNs = NNs_;
}

void addGaussNoise( Eigen::MatrixXd & M, double scale){

    std::cout<< "Adding Gaussian Noise with std "<< scale << "% of the bounding box" << std::endl;

    // find min and max point values of a mesh
    Eigen::MatrixXd max = M.colwise().maxCoeff();
    Eigen::MatrixXd min = M.colwise().minCoeff();

    // find distances
    double Xdist = max(0) - min(0);
    double Ydist = max(1) - min(1);
    double Zdist = max(2) - min(2);
    double mean = 0;
    double Xnoise ;
    double Ynoise ;
    double Znoise ;

    // create distributions
    std::default_random_engine generator;
    std::normal_distribution<double> Xdistribution(mean,Xdist*scale);
    std::normal_distribution<double> Ydistribution(mean,Ydist*scale);
    std::normal_distribution<double> Zdistribution(mean,Zdist*scale);

    Eigen::MatrixXd noise(1,3);

    // for all points add noise
    for (int i=0; i<M.rows(); i++) {
        // noise values for each dimension
        Xnoise = Xdistribution(generator);
        Ynoise = Ydistribution(generator);
        Znoise = Zdistribution(generator);
        noise << Xnoise, Ynoise, Znoise;

        M.row(i) = M.row(i) + noise;

        noise.setZero();
    }
    
}


void subsampling(Eigen::MatrixXd & M, int numberOfSamples){
    std::cout<< "Subsampling " << std::endl;

    // create distribution
    std::random_device rand_dev;
    std::mt19937 generator(rand_dev());
    std::uniform_int_distribution<int>  distr(0, M.rows()-1);

    // initialize matrix containing sampled points
    Eigen::MatrixXd sampledPoints(numberOfSamples,3);
    
    for (int i=0; i<numberOfSamples; i++) {
        // pick an index
        int index = distr(generator);
        // add sampled point
        sampledPoints.row(i) = M.row(index);
    }
    M = sampledPoints;
    std::cout << "Mesh dimensions (after) " << M.rows() << " by " << M.cols() << std::endl;
}

Eigen::Matrix3d getRotationMatrix(float xAngle, float yAngle, float zAngle) {
    // rotation on the x axis
    Eigen::Matrix3d Rx ;
    Rx << 1, 0,           0,
          0, cos(xAngle), -sin(xAngle),
          0, sin(xAngle), cos(xAngle) ;
    // rotation on the y axis
    Eigen::Matrix3d Ry ;
    Ry << cos(yAngle),  0, sin(yAngle),
          0,            1, 0,
          -sin(yAngle), 0, cos(yAngle) ;
    // rotation on the z axis
    Eigen::Matrix3d Rz ;
    Rz << cos(zAngle), -sin(zAngle), 0,
          sin(zAngle), cos(zAngle),  0,
          0,           0,            1 ;

    Eigen::Matrix3d R ;
    R = Rz*Ry*Rx ;
    return R ;
}

void ICP_p2Plane(Eigen::MatrixXd & Q, Eigen::MatrixXd const P){

    std::cout<< "Point to plane " << std::endl;

    // for convergence condition
    double error = 10000;
    int iter = 0;
    double threshold = 0.000001;
    double previous_error = 100000;

    std::cout << "threshold " << threshold << std::endl;
    
    while ( error > threshold && iter < 50 && abs(previous_error-error) > threshold*0.01) {
        
        Eigen::MatrixXd source = Q;
        
        // matrix of nearest neighbors
        size_t numberOfNN = 1;
        Eigen::MatrixXd NNs = findNN(Q, P, numberOfNN);
        
        Eigen::MatrixXd destination = NNs;
        //rejectPoints(Q,NNs);

        //centroids
        Eigen::RowVector3d P_cen = NNs.colwise().sum() / NNs.rows();
        Eigen::RowVector3d Q_cen = Q.colwise().sum() / Q.rows();
        
        // center point clouds
        Eigen::MatrixXd Q_hat;
        Q_hat = Q.rowwise() - Q_cen;
        Eigen::MatrixXd P_hat;
        P_hat = NNs.rowwise() - P_cen;

        // find normals of P_hat
        Eigen::MatrixXd NNs_Normals;
        Eigen::MatrixXi F;
        calculate_vertex_normal_flann(P_hat, F, NNs_Normals);

        // transformations
        Eigen::Matrix4d T;
        T = point2planeTransform(Q_hat,P_hat,NNs_Normals);

        // transform Q points in homogeneous coordinates
        Eigen::MatrixXd Q_hom = Eigen::MatrixXd::Ones(source.rows(),source.cols()+1);
        Q_hom.leftCols(3) = source;
        
        // apply rigid transformation to Q
        Eigen::MatrixXd movedPoints = (T*Q_hom.transpose()).transpose();
        movedPoints = movedPoints.array().colwise() / movedPoints.array().col(3);
        movedPoints = movedPoints.leftCols(3);
        
        // point to point measure
        previous_error = error;
        error = ((destination.array() - movedPoints.array()).pow(2)).sum() / movedPoints.rows();
        std::cout <<"iter " << iter <<" error " << error << ", error difference " << abs(previous_error-error) << std::endl;
        
        Q = movedPoints;
        iter += 1;
    }
    
}


Eigen::Matrix4d point2planeTransform(Eigen::MatrixXd & Q, Eigen::MatrixXd const P, Eigen::MatrixXd & NNs_Normals){
    // Q : mesh to move
    // P : destination mesh
    // NNs_Normals: normals of P points

    
    Eigen::Matrix4d T = Eigen::MatrixXd::Identity(4,4); // homogeneous rigid transformation matrix
    
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(P.rows(), P.cols() + NNs_Normals.cols());
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(Q.rows(), 1 );
    double a_i1;
    double a_i2;
    double a_i3;
    double nx;
    double ny;
    double nz;

    // build A matrix and b vector
    for (int i=0; i<P.rows(); i++) {

        nx = NNs_Normals.row(i).x();
        ny = NNs_Normals.row(i).y();
        nz = NNs_Normals.row(i).z();
        
        a_i1 = nz*Q.row(i).y() - ny*Q.row(i).z();
        a_i2 = nx*Q.row(i).z() - nz*Q.row(i).x();
        a_i3 = ny*Q.row(i).x() - nx*Q.row(i).y();

        A.row(i) << a_i1, a_i2, a_i3, nx, ny, nz;
        b.row(i) << nx*P.row(i).x() + ny*P.row(i).y() + nz*P.row(i).z() - nx*Q.row(i).x() - ny*Q.row(i).y() - nz*Q.row(i).z();
    }

    // SVD decomposition
    Eigen::JacobiSVD<Eigen::MatrixXd> A_svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd U = A_svd.matrixU();
    Eigen::MatrixXd S = A_svd.singularValues().asDiagonal();
    Eigen::MatrixXd V = A_svd.matrixV();

    // to solve Ax=b system -> x = pseudoA*b
    Eigen::MatrixXd pseudoA = V*S.inverse()*U.transpose();
    Eigen::MatrixXd x = pseudoA*b;
    // x = (xAngle,yAngle,zAngle,tx,ty,tz)
    
    x(0) = x(0)*PI/180;
    x(1) = x(1)*PI/180;
    x(2) = x(2)*PI/180;
    Eigen::Matrix3d R = getRotationMatrix(x(0), x(1), x(2));
    Eigen::MatrixXd t(3,1);
    t << x(3),x(4),x(5);
    
    T.topLeftCorner(3, 3) = R;
    T.topRightCorner(3, 1) = t;
    
    return T;
}
