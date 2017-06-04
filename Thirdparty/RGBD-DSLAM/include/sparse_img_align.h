#ifndef  SPARSE_IMG_ALIGN_H_
#define SPARSE_IMG_ALIGN_H_

#include <vikit/nlls_solver.h>
#include <vikit/performance_monitor.h>
#include "global.h"

namespace vk {
class AbstractCamera;
}

namespace RGBD_DSLAM {

class Feature;

/// Optimize the pose of the frame by minimizing the photometric error of feature patches.
class SparseImgAlign : public vk::NLLSSolver<6, SE3>
{
  static const int patch_halfsize_ = 2;
  static const int patch_size_ = 2*patch_halfsize_;
  static const int patch_area_ = patch_size_*patch_size_;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  cv::Mat resimg_;

  SparseImgAlign(
      int n_levels,
      int min_level,
      int n_iter,
      Method method,
      bool display,
      bool verbose);

  size_t run(
      FramePtr ref_frame,
      FramePtr cur_frame,
      Sophus::SE3& T_cur_ref);

  /// Return fisher information matrix, i.e. the Hessian of the log-likelihood
  /// at the converged state.
  Matrix<double, 6, 6> getFisherInformation();

protected:
  FramePtr ref_frame_;            //!< reference frame, has depth for gradient pixels.
  FramePtr cur_frame_;            //!< only the image is known!
  int level_;                     //!< current pyramid level on which the optimization runs.
  bool display_;                  //!< display residual image.
  int max_level_;                 //!< coarsest pyramid level for the alignment.
  int min_level_;                 //!< finest pyramid level for the alignment.

  // cache:
  Matrix<double, 6, Dynamic, ColMajor> jacobian_cache_;
  bool have_ref_patch_cache_;
  cv::Mat ref_patch_cache_;
  std::vector<bool> visible_fts_;

  void precomputeReferencePatches();
  virtual double computeResiduals(const SE3& model, bool linearize_system, bool compute_weight_scale = false);
  virtual int solve();
  virtual void update (const ModelType& old_model, ModelType& new_model);
  virtual void startIteration();
  virtual void finishIteration();
};

} // namespace RGBD_DSLAM

#endif // SPARSE_IMG_ALIGN_H_
