/*
 * FeatureExtractor.cpp
 *
 *  Created on: Feb 07, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "lesta/core/FeatureExtractor.h"
#include <Eigen/Dense>

namespace lesta {

FeatureExtractor::FeatureExtractor(const Config &cfg) : cfg(cfg) {}

void FeatureExtractor::ensureFeatureLayers(HeightMap &map) {
  //对应论文中的Terrain Attributes
  // Basic feature layers
  map.addLayer(layers::Feature::STEP);
  map.addLayer(layers::Feature::SLOPE);
  map.addLayer(layers::Feature::ROUGHNESS);
  map.addLayer(layers::Feature::CURVATURE);

  // Layers for visualization of normal vector
  map.addLayer(layers::Feature::NORMAL_X);
  map.addLayer(layers::Feature::NORMAL_Y);
  map.addLayer(layers::Feature::NORMAL_Z);
}

void FeatureExtractor::extractFeatures(HeightMap &map) {

  ensureFeatureLayers(map);
  // TODO: implement feature extraction for entire map (less efficient)
}

void FeatureExtractor::extractFeatures(
    HeightMap &map,
    const std::vector<grid_map::Index> &measured_indices) {
  //measured_indices：一个索引列表，包含了本帧“有效”被测量到的栅格位置。
  ensureFeatureLayers(map);

  for (const auto &index : measured_indices) {
    if (map.isEmptyAt(index))
      continue;

    const auto &neighbors = map.getNeighborHeights(index, cfg.pca_radius);
    if (neighbors.size() < 4)
      continue;

    // 1. Compute Covariance Matrix
    Eigen::Matrix3d covariance;
    Eigen::Vector3d sum_neighbors(Eigen::Vector3d::Zero());
    Eigen::Matrix3d squared_sum_neighbors(Eigen::Matrix3d::Zero());
    for (const auto &neighbor : neighbors) {
      sum_neighbors += neighbor;
      squared_sum_neighbors.noalias() += neighbor * neighbor.transpose();
    }
    const auto mean_neighbors = sum_neighbors / neighbors.size();
    covariance = squared_sum_neighbors / neighbors.size() -
                 mean_neighbors * mean_neighbors.transpose();

    // Check if covariance matrix is degenerated using trace
    if (covariance.trace() < std::numeric_limits<float>::epsilon())
      continue;

    // Compute Eigenvectors and Eigenvalues
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
    solver.computeDirect(covariance, Eigen::DecompositionOptions::ComputeEigenvectors);
    const auto &eigenvectors = solver.eigenvectors();
    const auto &eigenvalues = solver.eigenvalues();

    // Line feature: second eigen value is near zero -> normal is not defined
    if (eigenvalues(1) < 1e-8)
      continue;

    // Check direction of the normal vector and flip the sign towards the user defined
    // direction.
    //最小特征值对应的特征向量（col(0)）即最优法向方向。
    Eigen::Vector3d normal_vector = eigenvectors.col(0);
    Eigen::Vector3d positive_normal_vector(Eigen::Vector3d::UnitZ());
    if (normal_vector.dot(positive_normal_vector) < 0.0)
      normal_vector *= -1;

    // Calculate step
    //用 maxZ - minZ 表示区间的高差，反映该点周围地形是否有台阶或高度突变。
    auto minMax =
        std::minmax_element(neighbors.begin(),
                            neighbors.end(),
                            [](const Eigen::Vector3d &lhs, const Eigen::Vector3d &rhs) {
                              return lhs(2) < rhs(2); // Compare z-components.
                            });

    double minZ = (*minMax.first)(2);  // Minimum z-component.
    double maxZ = (*minMax.second)(2); // Maximum z-component.
    map.at(layers::Feature::STEP, index) = maxZ - minZ;

    map.at(layers::Feature::SLOPE, index) =
        std::acos(std::abs(normal_vector(2))) * 180 / M_PI;
    map.at(layers::Feature::ROUGHNESS, index) = std::sqrt(eigenvalues(0));
    map.at(layers::Feature::CURVATURE, index) =
        std::abs(eigenvalues(0) / covariance.trace());
    map.at(layers::Feature::NORMAL_X, index) = normal_vector(0);
    map.at(layers::Feature::NORMAL_Y, index) = normal_vector(1);
    map.at(layers::Feature::NORMAL_Z, index) = normal_vector(2);
  }
}

} // namespace lesta
