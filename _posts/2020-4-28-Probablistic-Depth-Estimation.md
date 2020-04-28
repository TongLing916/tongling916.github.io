---
layout:     post
title:      "Probablistic Depth Estimation"
date:       2020-4-28
author:     Tong
catalog: true
tags:
    - Reconstruction
---

### [Video-based, Real-Time Multi View Stereo](http://www.george-vogiatzis.org/) [^Vogiatzis11]

#### Abstract

We investigate the problem of obtaining a dense reconstruction in real-time, from a live video stream. In recent years, Multi-view stereo (MVS) has received considerable attention and a number of methods have been proposed. However, most methods operate under the assumption of a relatively sparse set of still images as input and unlimited computation time. Video based MVS has received less attention despite the fact that video sequences offer significant benefits in terms of usability of MVS systems. In this paper we propose a novel video based MVS algorithm that is suitable for real-time, interactive 3d modeling with a hand-held camera. The key idea is a per-pixel, probabilistic depth estimation scheme that updates posterior depth distributions with every new frame. The current implementation is capable of updating 15 million distributions per second. We evaluate the proposed method against the state-of-the-art real-time MVS method and show improvement in terms of accuracy.

#### Gaussian + Uniform Mixture Model


### Application - [SVO](https://github.com/uzh-rpg/rpg_svo)

#### Initialize Seed

```c++
struct Seed
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static int batch_counter;
  static int seed_counter;
  int batch_id;                //!< Batch id is the id of the keyframe for which the seed was created.
  int id;                      //!< Seed ID, only used for visualization.
  Feature* ftr;                //!< Feature in the keyframe for which the depth should be computed.
  float a;                     //!< a of Beta distribution: When high, probability of inlier is large.
  float b;                     //!< b of Beta distribution: When high, probability of outlier is large.
  float mu;                    //!< Mean of normal distribution.
  float z_range;               //!< Max range of the possible depth.
  float sigma2;                //!< Variance of normal distribution.
  Matrix2d patch_cov;          //!< Patch covariance in reference image.
  Seed(Feature* ftr, float depth_mean, float depth_min);
};

Seed::Seed(Feature* ftr, float depth_mean, float depth_min) :
    batch_id(batch_counter),
    id(seed_counter++),
    ftr(ftr),
    a(10),
    b(10),
    mu(1.0/depth_mean),
    z_range(1.0/depth_min),
    sigma2(z_range*z_range/36)
{}
```

#### Search along Epipolar Line


```c++
// we are using inverse depth coordinates
float z_inv_min = it->mu + sqrt(it->sigma2);
float z_inv_max = max(it->mu - sqrt(it->sigma2), 0.00000001f);
double z;
if(!matcher_.findEpipolarMatchDirect(
    *it->ftr->frame, *frame, *it->ftr, 1.0/it->mu, 1.0/z_inv_min, 1.0/z_inv_max, z))
{
  it->b++; // increase outlier probability when no match was found
  ++it;
  ++n_failed_matches;
  continue;
}
```

#### Compute Uncertainty

```c++
const double focal_length = frame->cam_->errorMultiplier2();
double px_noise = 1.0;  // assume measurement noise is 1 pixel
double px_error_angle = atan(px_noise/(2.0*focal_length))*2.0; // law of chord (sehnensatz)
```

```c++
// compute tau
double tau = computeTau(T_ref_cur, it->ftr->f, z, px_error_angle);
```

```c++
double DepthFilter::computeTau(
      const SE3& T_ref_cur,
      const Vector3d& f,
      const double z,
      const double px_error_angle)
{
  Vector3d t(T_ref_cur.translation());
  Vector3d a = f*z-t;
  double t_norm = t.norm();
  double a_norm = a.norm();
  double alpha = acos(f.dot(t)/t_norm); // dot product
  double beta = acos(a.dot(-t)/(t_norm*a_norm)); // dot product
  double beta_plus = beta + px_error_angle;
  double gamma_plus = PI-alpha-beta_plus; // triangle angles sum to PI
  double z_plus = t_norm*sin(beta_plus)/sin(gamma_plus); // law of sines
  return (z_plus - z); // tau
}
```

#### Update Seed

```c++
double tau_inverse = 0.5 * (1.0/max(0.0000001, z-tau) - 1.0/(z+tau));

// update the estimate
updateSeed(1./z, tau_inverse*tau_inverse, &*it);
```

```c++
/// Bayes update of the seed, x is the measurement, tau2 the measurement uncertainty
void DepthFilter::updateSeed(const float x, const float tau2, Seed* seed)
{
  float norm_scale = sqrt(seed->sigma2 + tau2);
  if(std::isnan(norm_scale))
    return;
  boost::math::normal_distribution<float> nd(seed->mu, norm_scale);
  float s2 = 1./(1./seed->sigma2 + 1./tau2);
  float m = s2*(seed->mu/seed->sigma2 + x/tau2);
  float C1 = seed->a/(seed->a+seed->b) * boost::math::pdf(nd, x);
  float C2 = seed->b/(seed->a+seed->b) * 1./seed->z_range;
  float normalization_constant = C1 + C2;
  C1 /= normalization_constant;
  C2 /= normalization_constant;
  float f = C1*(seed->a+1.)/(seed->a+seed->b+1.) + C2*seed->a/(seed->a+seed->b+1.);
  float e = C1*(seed->a+1.)*(seed->a+2.)/((seed->a+seed->b+1.)*(seed->a+seed->b+2.))
          + C2*seed->a*(seed->a+1.0f)/((seed->a+seed->b+1.0f)*(seed->a+seed->b+2.0f));

  // update parameters
  float mu_new = C1*m+C2*seed->mu;
  seed->sigma2 = C1*(s2 + m*m) + C2*(seed->sigma2 + seed->mu*seed->mu) - mu_new*mu_new;
  seed->mu = mu_new;
  seed->a = (e-f)/(f-e/f);
  seed->b = seed->a*(1.0f-f)/f;
}
```

### Create Point

```c++
// if the seed has converged, we initialize a new candidate point and remove the seed
if(sqrt(it->sigma2) < it->z_range/options_.seed_convergence_sigma2_thresh)
{
  Vector3d xyz_world(it->ftr->frame->T_f_w_.inverse() * (it->ftr->f * (1.0/it->mu)));
  Point* point = new Point(xyz_world, it->ftr);
  it->ftr->point = point;
  {
    seed_converged_cb_(point, it->sigma2); // put in candidate list
  }
  it = seeds_.erase(it);
}
```

### Literature

[^Vogiatzis11]: G. Vogiatzis and C. Hernandez, _Video-based, real-time multi-view stereo_, Image and Vision Computing, vol. 29, no. 7, 2011.
