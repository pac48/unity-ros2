#include "interface.h"
#include "generated/tmp.h"

int main() {
     sensor_msgs_Image input;
    std::intptr_t handle = NULL;
    Publish(handle, "sensor_msgs/msg/Image", "test_topic", (void *) &input);
    return 0;
}