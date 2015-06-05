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
 * Adjusted by Michael Goerner in 2015 to extract indices instead of a point cloud
 */

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>
#include <boost/make_shared.hpp>

struct ExtractLargestClusterIndices
{
  static void declare_params(tendrils& params) { }
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare(&ExtractLargestClusterIndices::clusters_, "clusters", "Clusters as indices.");
    outputs.declare(&ExtractLargestClusterIndices::output_, "output", "Indices of filtered cloud.");
  }

  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs) { }

  int process(const tendrils& inputs, const tendrils& outputs)
  {
    size_t largest = 0;
    for (size_t i = 1; i < clusters_->size(); i++)
    {
        if( (*clusters_)[i].indices.size() > (*clusters_)[largest].indices.size() )
        {
          largest = i;
        }
    }

    *output_= boost::make_shared<::pcl::PointIndices>((*clusters_)[largest]);

    return ecto::OK;
  }

  ecto::spore<ecto::pcl::Clusters> clusters_;
  ecto::spore<boost::shared_ptr<::pcl::PointIndices const>> output_;
};

ECTO_CELL(uos_ecto_cells, ExtractLargestClusterIndices,
          "ExtractLargestClusterIndices", "Extract the largest cluster.");

