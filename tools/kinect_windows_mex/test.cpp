
//
// initializes the Kinect NUI runtime, enables skeleton tracking, opens the color stream, reports success or failure at each step, then shuts down. It's the standard init/enable/open/shutdown sequence any Kinect NUI program has to follow. dictated by the API's own shape.

#include <Windows.h>
#include <NuiApi.h>
#include <stdio.h>

int main()
{
    HRESULT hr = NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX |
                                NUI_INITIALIZE_FLAG_USES_SKELETON |
                                NUI_INITIALIZE_FLAG_USES_COLOR);
    if (FAILED(hr))
    {
        printf("NuiInitialize failed (HRESULT 0x%08X). Is a Kinect connected?\n", hr);
        return 1;
    }
    printf("NuiInitialize succeeded.\n");

    HANDLE hNextSkeletonEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    hr = NuiSkeletonTrackingEnable(hNextSkeletonEvent, 0);
    if (FAILED(hr))
        printf("NuiSkeletonTrackingEnable failed (HRESULT 0x%08X).\n", hr);
    else
        printf("Skeleton tracking enabled.\n");

    HANDLE hColorStream = NULL;
    hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480,
                             0, 2, NULL, &hColorStream);
    if (FAILED(hr))
        printf("NuiImageStreamOpen (color) failed (HRESULT 0x%08X).\n", hr);
    else
        printf("Color stream opened.\n");

    NuiShutdown();
    printf("NuiShutdown complete.\n");
    return 0;
}
