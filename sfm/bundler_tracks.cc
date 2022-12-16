/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>
#include <set>

#include "core/image_tools.h"
#include "core/image_drawing.h"
#include "sfm/bundler_tracks.h"

SFM_NAMESPACE_BEGIN
SFM_BUNDLER_NAMESPACE_BEGIN

/*
 * Merges tracks and updates viewports accordingly.
 */
void
unify_tracks(int view1_tid, int view2_tid,
    TrackList* tracks, ViewportList* viewports)
{
    /* 将小的track合并进在较大的跟踪;. */
    if (tracks->at(view1_tid).features.size()
        < tracks->at(view2_tid).features.size())
        std::swap(view1_tid, view2_tid); // 如果view1_tid更小，则进行交换

    Track& track1 = tracks->at(view1_tid);
    Track& track2 = tracks->at(view2_tid);

    for (std::size_t k = 0; k < track2.features.size(); ++k) // 遍历track2的所有元素
    {
        int const view_id = track2.features[k].view_id;
        int const feat_id = track2.features[k].feature_id;
        viewports->at(view_id).track_ids[feat_id] = view1_tid; // 将第二个特征点对应的trackid更新为第一个特征点的trackid
    }
    track1.features.insert(track1.features.end(),
        track2.features.begin(), track2.features.end()); // 同时在track1的features添加track2的features
    /* Free old track's memory. clear() does not work. */
    track2.features = FeatureReferenceList(); // 清除老的track信息
}

/* ---------------------------------------------------------------- */

void
Tracks::compute (PairwiseMatching const& matching,
    ViewportList* viewports, TrackList* tracks)
{
    /* Args:
     *  matching:元素大小为有n（n-1）/2，元素为：两两视图的索引id，特征匹配点对的索引id
     *  viewports: 保存了视图信息
     *  tracks： 匹配的特征点之间的链接关系
     * */
    for (std::size_t i = 0; i < viewports->size(); ++i)
    {
        Viewport& viewport = viewports->at(i);
        viewport.track_ids.resize(viewport.features.positions.size(), -1); // 对视图的每个特征点点都初始化一个tracks，默认值为-1
    }

    /* 打印状态信息 */
    if (this->opts.verbose_output)
        std::cout << "Propagating track IDs..." << std::endl;

    /* 遍历所有成对的点对并且构造tracks */
    tracks->clear();
    for (std::size_t i = 0; i < matching.size(); ++i) // 遍历3个视图
    {
        TwoViewMatching const& tvm = matching[i];
        Viewport& viewport1 = viewports->at(tvm.view_1_id); //
        Viewport& viewport2 = viewports->at(tvm.view_2_id);

        /* 遍历连接图中的匹配的特征点对 */
        for (std::size_t j = 0; j < tvm.matches.size(); ++j)
        {
            CorrespondenceIndex idx = tvm.matches[j];
            int const view1_tid = viewport1.track_ids[idx.first]; // 获取到第一个特征点所对应的trackid
            int const view2_tid = viewport2.track_ids[idx.second]; // 获取到第二个特征点所对应的trackid
            if (view1_tid == -1 && view2_tid == -1)
            {
                /* 如果两个特征点都没有分配到track,则为这两个特征点创建同一个track */
                viewport1.track_ids[idx.first] = tracks->size(); // 用track的数量作为值
                viewport2.track_ids[idx.second] = tracks->size();
                tracks->push_back(Track()); // 先压进vector一个初始的track
                tracks->back().features.push_back( // ->features有2个元素，每个元素放置对应特征点的视图id和对应特征点的id
                    FeatureReference(tvm.view_1_id, idx.first));
                tracks->back().features.push_back(
                    FeatureReference(tvm.view_2_id, idx.second));
            }
            else if (view1_tid == -1 && view2_tid != -1)  // 第一个特征点没有tracks，第二个特征点有tracks,将第一个特征点的tracks更新为第二个
            {
                /* 更新第一个特征点的tracks */
                viewport1.track_ids[idx.first] = view2_tid;
                tracks->at(view2_tid).features.push_back( /* 加入第一个特征点的信息到tracks中去，此时tracks对应元素的特征点个数大于2个 */
                    FeatureReference(tvm.view_1_id, idx.first));
            }
            else if (view1_tid != -1 && view2_tid == -1)  // 同上
            {
                /* Propagate track ID from second to first view. */
                viewport2.track_ids[idx.second] = view1_tid;
                tracks->at(view1_tid).features.push_back(
                    FeatureReference(tvm.view_2_id, idx.second));
            }
            else if (view1_tid == view2_tid)
            {
                /* 已经合成了track了，则不做处理 */
            }
            else
            {
                /* 如果匹配的两个特征点对应的track id不一样，则将track进行融合
                 * A track ID is already associated with both ends of a match,
                 * however, is not consistent. Unify tracks.
                 */
                unify_tracks(view1_tid, view2_tid, tracks, viewports);
            }
        }
    }

    /* 过滤掉一些track. */
    if (this->opts.verbose_output)
        std::cout << "Removing tracks with conflicts..." << std::flush;

    // 删除不合理的track(同一个track,包含同一副图像中的多个特征点）
    std::size_t const num_invalid_tracks = this->remove_invalid_tracks(viewports, tracks);
    if (this->opts.verbose_output)
        std::cout << " deleted " << num_invalid_tracks << " tracks." << std::endl;

    /* 对每个track计算的特征点计算颜色 */
    if (this->opts.verbose_output)
        std::cout << "Colorizing tracks..." << std::endl;
    for (std::size_t i = 0; i < tracks->size(); ++i) // 遍历所有的track
    {
        Track& track = tracks->at(i);
        math::Vec4f color(0.0f, 0.0f, 0.0f, 0.0f);
        for (std::size_t j = 0; j < track.features.size(); ++j) // 遍历一条track上的所有特征点
        {
            FeatureReference const& ref = track.features[j];
            FeatureSet const& features = viewports->at(ref.view_id).features;
            math::Vec3f const feature_color(features.colors[ref.feature_id]); //计算特征点的颜色
            color += math::Vec4f(feature_color, 1.0f);
        }
        track.color[0] = static_cast<uint8_t>(color[0] / color[3] + 0.5f); // 颜色也归一化？
        track.color[1] = static_cast<uint8_t>(color[1] / color[3] + 0.5f);
        track.color[2] = static_cast<uint8_t>(color[2] / color[3] + 0.5f);
    }
}

/* ---------------------------------------------------------------- */

int
Tracks::remove_invalid_tracks (ViewportList* viewports, TrackList* tracks)
{
    /*
     * Detect invalid tracks where a track contains no features, or
     * multiple features from a single view.
     */
    // 1.过滤掉那些不包含特征的track 2. 过滤掉一个track链接一个图像中多个特征点，即一个track在一张图像中只能连接一个特征点
    std::vector<bool> delete_tracks(tracks->size());
    int num_invalid_tracks = 0;
    for (std::size_t i = 0; i < tracks->size(); ++i)
    {
        if (tracks->at(i).features.empty()) {
            delete_tracks[i] = true;
            continue;
        }

        std::set<int> view_ids; // 保存每一个track横跨的图像id
        for (std::size_t j = 0; j < tracks->at(i).features.size(); ++j) // 遍历track路径上的所有特征点
        {
            FeatureReference const& ref = tracks->at(i).features[j];
            if (view_ids.insert(ref.view_id).second == false) { // 如果set中已经有了该元素,重复插入返回false
                num_invalid_tracks += 1; // 无效的track++
                delete_tracks[i] = true;
                break;
            }
        }
    }

    /* Create a mapping from old to new track IDs. */
    std::vector<int> id_mapping(delete_tracks.size(), -1);
    int valid_track_counter = 0;
    for (std::size_t i = 0; i < delete_tracks.size(); ++i)
    {
        if (delete_tracks[i]) // 如果是需要删除的track
            continue;
        id_mapping[i] = valid_track_counter;
        valid_track_counter += 1;
    }
    /* 修复存储在视图中的track ids*/
    for (std::size_t i = 0; i < viewports->size(); ++i) // 遍历所有视图
    {
        std::vector<int>& track_ids = viewports->at(i).track_ids;
        for (std::size_t j = 0; j < track_ids.size(); ++j)
            if (track_ids[j] >= 0)
                track_ids[j] = id_mapping[track_ids[j]];
    }

    /* Clean the tracks from the vector. */
    math::algo::vector_clean(delete_tracks, tracks);

    return num_invalid_tracks;
}


SFM_BUNDLER_NAMESPACE_END
SFM_NAMESPACE_END
