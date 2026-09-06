#include <limits>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "yolo_world_detector/yolo_world_utils.hpp"

namespace
{

yolo_world_detector::LetterboxResult identityLetterbox()
{
    yolo_world_detector::LetterboxResult result;
    result.scale = 1.0F;
    return result;
}

TEST(Letterbox, RestoresWideImageCoordinates)
{
    const cv::Mat image(320, 640, CV_8UC3, cv::Scalar::all(0));
    const auto letterboxed = yolo_world_detector::letterbox(image, 640, 640);
    ASSERT_EQ(letterboxed.pad_top, 160);

    std::vector<float> output(14U, 0.0F);
    output[0] = 320.0F;
    output[1] = 320.0F;
    output[2] = 200.0F;
    output[3] = 100.0F;
    output[4] = 0.9F;
    const auto detections = yolo_world_detector::decodeWorldOutput(
        output.data(), {1, 14, 1}, 10, 0.25F, 0.45F, letterboxed, 640, 320);
    ASSERT_EQ(detections.size(), 1U);
    EXPECT_FLOAT_EQ(detections[0].box.x, 220.0F);
    EXPECT_FLOAT_EQ(detections[0].box.y, 110.0F);
    EXPECT_FLOAT_EQ(detections[0].box.width, 200.0F);
    EXPECT_FLOAT_EQ(detections[0].box.height, 100.0F);
}

TEST(Decode, SupportsConfirmedAttributesFirstLayoutAndClassAwareNms)
{
    const std::size_t candidates = 3U;
    std::vector<float> output(14U * candidates, 0.0F);
    const auto set = [&output, candidates](std::size_t candidate,
                                           std::size_t attribute, float value) {
        output[attribute * candidates + candidate] = value;
    };
    for (std::size_t index = 0; index < candidates; ++index)
    {
        set(index, 0, 50.0F + static_cast<float>(index));
        set(index, 1, 50.0F);
        set(index, 2, 20.0F);
        set(index, 3, 20.0F);
    }
    set(0, 4, 0.90F);
    set(1, 4, 0.80F);
    set(2, 5, 0.70F);

    const auto detections = yolo_world_detector::decodeWorldOutput(
        output.data(), {1, 14, 3}, 10, 0.25F, 0.45F, identityLetterbox(),
        100, 100);
    ASSERT_EQ(detections.size(), 2U);
    EXPECT_EQ(detections[0].class_id, 0);
    EXPECT_EQ(detections[1].class_id, 1);
}

TEST(Decode, SupportsCandidateFirstLayout)
{
    std::vector<float> output(2U * 14U, 0.0F);
    output[0] = 50.0F;
    output[1] = 50.0F;
    output[2] = 20.0F;
    output[3] = 10.0F;
    output[4] = 0.9F;
    output[14] = 75.0F;
    output[15] = 70.0F;
    output[16] = 10.0F;
    output[17] = 20.0F;
    output[19] = 0.8F;

    const auto detections = yolo_world_detector::decodeWorldOutput(
        output.data(), {1, 2, 14}, 10, 0.25F, 0.45F, identityLetterbox(),
        100, 100);
    ASSERT_EQ(detections.size(), 2U);
    EXPECT_EQ(detections[0].class_id, 0);
    EXPECT_EQ(detections[1].class_id, 1);
}

TEST(Decode, AppliesThresholdAndRejectsInvalidBoxes)
{
    std::vector<float> output(14U * 2U, 0.0F);
    const auto set = [&output](std::size_t candidate, std::size_t attribute,
                               float value) {
        output[attribute * 2U + candidate] = value;
    };
    set(0, 0, 50.0F);
    set(0, 1, 50.0F);
    set(0, 2, 20.0F);
    set(0, 3, 20.0F);
    set(0, 4, 0.20F);
    set(1, 0, std::numeric_limits<float>::quiet_NaN());
    set(1, 1, 50.0F);
    set(1, 2, 20.0F);
    set(1, 3, 20.0F);
    set(1, 4, 0.90F);

    EXPECT_TRUE(yolo_world_detector::decodeWorldOutput(
                    output.data(), {1, 14, 2}, 10, 0.25F, 0.45F,
                    identityLetterbox(), 100, 100)
                    .empty());
}

TEST(Decode, RejectsWrongShape)
{
    std::vector<float> output(14U, 0.0F);
    EXPECT_THROW(yolo_world_detector::decodeWorldOutput(
                     output.data(), {1, 13, 1}, 10, 0.25F, 0.45F,
                     identityLetterbox(), 100, 100),
                 std::runtime_error);
}

TEST(TemporalHold, IsDisabledByDefault)
{
    yolo_world_detector::TemporalHoldState state;
    std::vector<yolo_world_detector::Detection> detections = {
        {0, 0.9F, cv::Rect2f(10.0F, 10.0F, 20.0F, 20.0F)}};
    bool used_hold = true;
    const auto first = yolo_world_detector::applyTemporalHold(
        detections, 0U, 0.0, 1.0, &state, &used_hold);
    EXPECT_EQ(first.size(), 1U);
    EXPECT_FALSE(used_hold);

    const auto second = yolo_world_detector::applyTemporalHold(
        {}, 0U, 0.0, 1.1, &state, &used_hold);
    EXPECT_TRUE(second.empty());
    EXPECT_FALSE(used_hold);
}

TEST(TemporalHold, ReusesRecentCompleteDetectionsForSparseFrame)
{
    yolo_world_detector::TemporalHoldState state;
    std::vector<yolo_world_detector::Detection> complete = {
        {0, 0.9F, cv::Rect2f(10.0F, 10.0F, 20.0F, 20.0F)},
        {0, 0.8F, cv::Rect2f(50.0F, 10.0F, 20.0F, 20.0F)},
        {0, 0.7F, cv::Rect2f(90.0F, 10.0F, 20.0F, 20.0F)}};
    bool used_hold = true;
    const auto first = yolo_world_detector::applyTemporalHold(
        complete, 3U, 0.5, 10.0, &state, &used_hold);
    EXPECT_EQ(first.size(), 3U);
    EXPECT_FALSE(used_hold);

    const auto held = yolo_world_detector::applyTemporalHold(
        {complete.front()}, 3U, 0.5, 10.4, &state, &used_hold);
    EXPECT_EQ(held.size(), 3U);
    EXPECT_TRUE(used_hold);

    const auto expired = yolo_world_detector::applyTemporalHold(
        {complete.front()}, 3U, 0.5, 10.6, &state, &used_hold);
    EXPECT_EQ(expired.size(), 1U);
    EXPECT_FALSE(used_hold);
}

}  // namespace

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
