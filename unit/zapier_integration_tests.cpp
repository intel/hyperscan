#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <zapier/zapier_integration.py>

using namespace testing;

class ZapierIntegrationTest : public Test {
protected:
    void SetUp() override {
        // Set up any necessary preconditions for the tests
    }

    void TearDown() override {
        // Clean up any resources used by the tests
    }
};

TEST_F(ZapierIntegrationTest, TriggerEvent) {
    // Mock the requests.post function
    MockFunction<requests::post> mock_post;
    EXPECT_CALL(mock_post, Call(_, _))
        .WillOnce(Return(MockResponse(200, R"({"status": "success"})")));

    // Call the trigger_event function
    auto response = trigger_event("test_event", {{"key", "value"}});

    // Verify the response
    EXPECT_EQ(response["status"], "success");
}

TEST_F(ZapierIntegrationTest, ActionCreateItem) {
    // Mock the requests.post function
    MockFunction<requests::post> mock_post;
    EXPECT_CALL(mock_post, Call(_, _))
        .WillOnce(Return(MockResponse(200, R"({"status": "success"})")));

    // Call the action_create_item function
    auto response = action_create_item("test_item", {{"key", "value"}});

    // Verify the response
    EXPECT_EQ(response["status"], "success");
}

TEST_F(ZapierIntegrationTest, ActionUpdateItem) {
    // Mock the requests.post function
    MockFunction<requests::post> mock_post;
    EXPECT_CALL(mock_post, Call(_, _))
        .WillOnce(Return(MockResponse(200, R"({"status": "success"})")));

    // Call the action_update_item function
    auto response = action_update_item("test_item_id", {{"key", "value"}});

    // Verify the response
    EXPECT_EQ(response["status"], "success");
}

TEST_F(ZapierIntegrationTest, ActionDeleteItem) {
    // Mock the requests.post function
    MockFunction<requests::post> mock_post;
    EXPECT_CALL(mock_post, Call(_, _))
        .WillOnce(Return(MockResponse(200, R"({"status": "success"})")));

    // Call the action_delete_item function
    auto response = action_delete_item("test_item_id");

    // Verify the response
    EXPECT_EQ(response["status"], "success");
}
