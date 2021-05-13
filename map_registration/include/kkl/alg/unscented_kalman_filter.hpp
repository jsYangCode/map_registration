/**
 * UnscentedKalmanFilterX.hpp
 * @author koide
 * 16/02/01
 **/
#ifndef KKL_UNSCENTED_KALMAN_FILTER_X_HPP
#define KKL_UNSCENTED_KALMAN_FILTER_X_HPP

#include <random>
#include <Eigen/Dense>

namespace kkl {
  namespace alg {

/**
 * @brief Unscented Kalman Filter class
 * @param T        scaler type
 * @param System   system class to be estimated
 */
template<typename T, class System>
class UnscentedKalmanFilterX {
  typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXt;
  typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> MatrixXt;
public:
  /**
   * @brief constructor
   * @param system               system to be estimated
   * @param state_dim            state vector dimension
   * @param input_dim            input vector dimension
   * @param measurement_dim      measurement vector dimension
   * @param process_noise        process noise covariance (state_dim x state_dim)
   * @param measurement_noise    measurement noise covariance (measurement_dim x measuremend_dim)
   * @param mean                 initial mean
   * @param cov                  initial covariance
   */
  UnscentedKalmanFilterX(const System& system, int state_dim, int input_dim, int measurement_dim, const MatrixXt& process_noise, const MatrixXt& measurement_noise, const VectorXt& mean, const MatrixXt& cov)
    : state_dim(state_dim),
    input_dim(input_dim),
    measurement_dim(measurement_dim),
    N(state_dim),//状态维度
    M(input_dim),//输入维度
    K(measurement_dim),//测量维度
    S(2 * state_dim + 1),// sigmapoints 需要是2 × state_dim + 1
    mean(mean),
    cov(cov),
    system(system),
    process_noise(process_noise),//预测噪声
    measurement_noise(measurement_noise),//测量噪声
    lambda(1),//计算sigma点的重要参数
    normal_dist(0.0, 1.0)
  {
    weights.resize(S, 1);//设置weights维度
    sigma_points.resize(S, N);//S是sigmapoints个数，N是状态维度
    ext_weights.resize(2 * (N + K) + 1, 1);//ext_weights: extened weights似乎包含了状态量和测量噪声
    ext_sigma_points.resize(2 * (N + K) + 1, N + K);
    expected_measurements.resize(2 * (N + K) + 1, K);

    // initialize weights for unscented filter
    weights[0] = lambda / (N + lambda);// lamda的作用   此处的系数是无及卡尔慢滤波的权重的标准计算过程   ，   lamda = n * a^2 -n,其中a是sigma点的散步程度，n是输出的维度
    for (int i = 1; i < 2 * N + 1; i++)
    {// 无迹卡尔慢滤波是在点的附近取2*n个点来计算
      weights[i] = 1 / (2 * (N + lambda));//计算采样点的权重
    }

    // weights for extended state space which includes error variances
    ext_weights[0] = lambda / (N + K + lambda);//todo 扩展的状态空间包含了测量值，此处需要理解下
    for (int i = 1; i < 2 * (N + K) + 1; i++)
    {
      ext_weights[i] = 1 / (2 * (N + K + lambda));
    }
  }

  /**
   * @brief predict
   * @param control  input vector
   */
  void predict(const VectorXt& control, Eigen::Vector3f odom_translation)
  {//计算预测状态的均值和方差 todo gc odom_translation
    // calculate sigma points
    ensurePositiveFinite(cov);//啥都没做
    computeSigmaPoints(mean, cov, sigma_points);
    for (int i = 0; i < S; i++)
    {
      sigma_points.row(i) = system.f(sigma_points.row(i), control,odom_translation);//计算sigmapoints经过状态转移方程计算后得到的一系列的点
    }

    const auto& R = process_noise;

    // unscented transform
    VectorXt mean_pred(mean.size());
    MatrixXt cov_pred(cov.rows(), cov.cols());

    mean_pred.setZero();
    cov_pred.setZero();
    for (int i = 0; i < S; i++)
    {
      mean_pred += weights[i] * sigma_points.row(i);//将所有的sigma点经过状态转移方程计算得到的点求加权均值
    }
    for (int i = 0; i < S; i++)
    {
      VectorXt diff = sigma_points.row(i).transpose() - mean;//todo mean是上一次的mean，这是为何？
      cov_pred += weights[i] * diff * diff.transpose();//计算协方差矩阵
    }
    cov_pred += R;//方差加上预测噪声

    mean = mean_pred;
    cov = cov_pred;
  }

  /**
   * @brief correct
   * @param measurement  measurement vector
   */
  void correct(const VectorXt& measurement)
  {
    // create extended state space which includes error variances
    VectorXt ext_mean_pred = VectorXt::Zero(N + K, 1);//扩展的协方差矩阵包含了噪声的方差
    MatrixXt ext_cov_pred = MatrixXt::Zero(N + K, N + K);
    ext_mean_pred.topLeftCorner(N, 1) = VectorXt(mean);//此处的mean是预测部分的均值
    ext_cov_pred.topLeftCorner(N, N) = MatrixXt(cov);//此处的cov是预测部分的方差
    ext_cov_pred.bottomRightCorner(K, K) = measurement_noise;//量测噪声

    ensurePositiveFinite(ext_cov_pred);
    computeSigmaPoints(ext_mean_pred, ext_cov_pred, ext_sigma_points);

    // unscented transform  todo：即根据测量方程和一系列的状态值计算一系列的输出值
    expected_measurements.setZero();
    for (int i = 0; i < ext_sigma_points.rows(); i++)
    {
      expected_measurements.row(i) = system.h(ext_sigma_points.row(i).transpose().topLeftCorner(N, 1));//依据测量方程计算依据预测状态的一系列sigmapoints计算得到的测量值
      expected_measurements.row(i) += VectorXt(ext_sigma_points.row(i).transpose().bottomRightCorner(K, 1));//todo 此处有点奇怪,133行没有添加测量噪声，此处应该等价的添加了
    }

    VectorXt expected_measurement_mean = VectorXt::Zero(K);
    for (int i = 0; i < ext_sigma_points.rows(); i++)
    {
      expected_measurement_mean += ext_weights[i] * expected_measurements.row(i);//计算均值
    }
    MatrixXt expected_measurement_cov = MatrixXt::Zero(K, K);//计算方差
    for (int i = 0; i < ext_sigma_points.rows(); i++)
    {
      VectorXt diff = expected_measurements.row(i).transpose() - expected_measurement_mean;
      expected_measurement_cov += ext_weights[i] * diff * diff.transpose();//预测的测量值的方差 todo 这里不是要加上测量噪声吗？
    }

    // calculated transformed covariance
    MatrixXt sigma = MatrixXt::Zero(N + K, K);
    for (int i = 0; i < ext_sigma_points.rows(); i++)
    {//计算协方差
      auto diffA = (ext_sigma_points.row(i).transpose() - ext_mean_pred);
      auto diffB = (expected_measurements.row(i).transpose() - expected_measurement_mean);
      sigma += ext_weights[i] * (diffA * diffB.transpose());
    }

    kalman_gain = sigma * expected_measurement_cov.inverse();//计算卡尔慢增益
    const auto& K = kalman_gain;

    VectorXt ext_mean = ext_mean_pred + K * (measurement - expected_measurement_mean);//在测量值与预测的测量值取折中作为其均值的期望
    MatrixXt ext_cov = ext_cov_pred - K * expected_measurement_cov * K.transpose();//融合后的方差

    mean = ext_mean.topLeftCorner(N, 1);
    cov = ext_cov.topLeftCorner(N, N);
  }

  /*			getter			*/
  const VectorXt& getMean() const { return mean; }
  const MatrixXt& getCov() const { return cov; }
  const MatrixXt& getSigmaPoints() const { return sigma_points; }

  System& getSystem() { return system; }
  const System& getSystem() const { return system; }
  const MatrixXt& getProcessNoiseCov() const { return process_noise; }
  const MatrixXt& getMeasurementNoiseCov() const { return measurement_noise; }

  const MatrixXt& getKalmanGain() const { return kalman_gain; }

  /*			setter			*/
  UnscentedKalmanFilterX& setMean(const VectorXt& m) { mean = m;			return *this; }
  UnscentedKalmanFilterX& setCov(const MatrixXt& s) { cov = s;			return *this; }

  UnscentedKalmanFilterX& setProcessNoiseCov(const MatrixXt& p) { process_noise = p;			return *this; }
  UnscentedKalmanFilterX& setMeasurementNoiseCov(const MatrixXt& m) { measurement_noise = m;	return *this; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  const int state_dim;
  const int input_dim;
  const int measurement_dim;

  const int N;
  const int M;
  const int K;
  const int S;

public:
  VectorXt mean;
    Eigen::Matrix<float , Eigen::Dynamic, Eigen::Dynamic> cov;

  System system;
  MatrixXt process_noise;		//
  MatrixXt measurement_noise;	//

  T lambda;
  VectorXt weights;

  MatrixXt sigma_points;

  VectorXt ext_weights;
  MatrixXt ext_sigma_points;
  MatrixXt expected_measurements;

private:
  /**
   * @brief compute sigma points
   * @param mean          mean
   * @param cov           covariance
   * @param sigma_points  calculated sigma points
   */
  void computeSigmaPoints(const VectorXt& mean, const MatrixXt& cov, MatrixXt& sigma_points)
  {
    const int n = mean.size();
    assert(cov.rows() == n && cov.cols() == n);
    //std::cout<<cov[1][2];
    Eigen::LLT<MatrixXt> llt;
    llt.compute((n + lambda) * cov);//recomputes the Cholesky decomposition A = LL^* = U^*U of \a matrix
    MatrixXt l = llt.matrixL();//a view of the lower triangular matrix L

    sigma_points.row(0) = mean;
    for (int i = 0; i < n; i++)
    {
      sigma_points.row(1 + i * 2) = mean + l.col(i);//可以参考probablistic tobotics的65面
      sigma_points.row(1 + i * 2 + 1) = mean - l.col(i);
    }
  }

  /**
   * @brief make covariance matrix positive finite
   * @param cov  covariance matrix
   */
  void ensurePositiveFinite(MatrixXt& cov)
  {
    return;
    const double eps = 1e-9;

    Eigen::EigenSolver<MatrixXt> solver(cov);
    MatrixXt D = solver.pseudoEigenvalueMatrix();
    MatrixXt V = solver.pseudoEigenvectors();
    for (int i = 0; i < D.rows(); i++)
    {
      if (D(i, i) < eps)
      {
        D(i, i) = eps;
      }
    }

    cov = V * D * V.inverse();
  }

public:
  MatrixXt kalman_gain;

  std::mt19937 mt;
  std::normal_distribution<T> normal_dist;
};

  }
}


#endif
