#include "pch.h"
#include <XrSceneLib/XrApp.h>

std::unique_ptr<Scene> TryCreateTitleScene(SceneContext& sceneContext);
std::unique_ptr<Scene> TryCreateMapScene(SceneContext& sceneContext);


int APIENTRY wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR lpCmdLine, _In_ int nCmdShow) {
    UNREFERENCED_PARAMETER(hInstance);
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);
    UNREFERENCED_PARAMETER(nCmdShow);

    try {
        CHECK_HRCMD(::CoInitializeEx(nullptr, COINIT_MULTITHREADED));
        auto on_exit = MakeScopeGuard([] { ::CoUninitialize(); });

        const std::vector<const char*> requiredExtensions = {
            XR_MSFT_UNBOUNDED_REFERENCE_SPACE_EXTENSION_NAME,
            XR_MSFT_SPATIAL_ANCHOR_EXTENSION_NAME,
            XR_MSFT_HAND_INTERACTION_EXTENSION_NAME,
#if (ENABLE_HP_CONTROLLER)
            XR_EXT_HP_MOTION_CONTROLLER_EXTENSION_NAME,
#endif
        };

        auto app = CreateXrApp({ "HpControllerTestClientUwp", 1 }, requiredExtensions);
        app->AddScene(TryCreateTitleScene(app->SceneContext()));
        app->AddScene(TryCreateMapScene(app->SceneContext()));
        app->Run();
    }
    catch (const std::exception& ex) {
        sample::Trace("Unhandled Exception: {}\n", ex.what());
        return 1;
    }
    catch (...) {
        sample::Trace(L"Unhandled Exception\n");
        return 1;
    }

    return 0;
}
