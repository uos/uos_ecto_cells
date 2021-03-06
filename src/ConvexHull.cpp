/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Extended by Michael Goerner in 2015 to export indices as well
 */

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>
#include <pcl/surface/convex_hull.h>

struct ConvexHull
{
  static void declare_params(tendrils& params)
  {
    params.declare(&ConvexHull::dimensionality_, "dimensionality", "Dimensionality of the data (valid: 2 and 3)", 3);
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare(&ConvexHull::indices_, "indices", "Indices of points of interest in input.");
    outputs.declare(&ConvexHull::output_, "output", "Points that form the the convex hull.");
    outputs.declare(&ConvexHull::output_indices_, "output_indices", "Indices of convex hull points.");
  }

  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs) { }

  template <typename Point>
  int process(const tendrils& inputs, const tendrils& outputs,
              boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
  {
    ::pcl::ConvexHull<Point> filter;
    typename ::pcl::PointCloud<Point>::Ptr cloud(new typename ::pcl::PointCloud<Point>);

    filter.setInputCloud(input);
    if(indices_.user_supplied())
      filter.setIndices(*indices_);
    filter.setDimension(*dimensionality_);
    filter.reconstruct(*cloud);

    auto output_indices= boost::make_shared<::pcl::PointIndices>();
    filter.getHullPointIndices(*output_indices);
    *output_indices_= output_indices;

    *output_ = ecto::pcl::xyz_cloud_variant_t(cloud);
    return ecto::OK;
  }

  ecto::spore<int> dimensionality_;
  ecto::spore< ::pcl::PointIndices::ConstPtr > indices_;
  ecto::spore<ecto::pcl::PointCloud> output_;
  ecto::spore< ::pcl::PointIndices::ConstPtr > output_indices_;
};

ECTO_CELL(uos_ecto_cells, ecto::pcl::PclCell<ConvexHull>,
          "ConvexHull", "Using libqhull library. (extended to produce indices)");
