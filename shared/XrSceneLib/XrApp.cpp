//*********************************************************
//    Copyright (c) Microsoft. All rights reserved.
//
//    Apache 2.0 License
//
//    You may obtain a copy of the License at
//    http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
//    implied. See the License for the specific language governing
//    permissions and limitations under the License.
//
//*********************************************************
#include "pch.h"

#include <XrUtility/XrEnumerate.h>
#include <XrUtility/XrToString.h>
#include <XrUtility/XrViewConfiguration.h>

#include <SampleShared/FileUtility.h>
#include <SampleShared/DxUtility.h>
#include <SampleShared/Trace.h>

#include "XrApp.h"
#include "CompositionLayers.h"
#include "SceneContext.h"

using namespace DirectX;
using namespace std::chrono_literals;

namespace {
    const std::vector<DXGI_FORMAT> SupportedColorSwapchainFormats = {
        DXGI_FORMAT_R8G8B8A8_UNORM,
        DXGI_FORMAT_B8G8R8A8_UNORM,
        DXGI_FORMAT_R8G8B8A8_UNORM_SRGB,
        DXGI_FORMAT_B8G8R8A8_UNORM_SRGB,
    };

    const std::vector<DXGI_FORMAT> SupportedDepthSwapchainFormats = {
        DXGI_FORMAT_D32_FLOAT,
        DXGI_FORMAT_D32_FLOAT_S8X24_UINT,
        DXGI_FORMAT_D24_UNORM_S8_UINT,
        DXGI_FORMAT_D16_UNORM,
    };

    const XrViewConfigurationType PrimaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;

    const std::vector<XrViewConfigurationType> SupportedViewConfigurationTypes = {
        XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
        XR_VIEW_CONFIGURATION_TYPE_SECONDARY_MONO_FIRST_PERSON_OBSERVER_MSFT,
    };

    const std::vector<XrEnvironmentBlendMode> SupportedEnvironmentBlendModes = {
        XR_ENVIRONMENT_BLEND_MODE_ADDITIVE,
        XR_ENVIRONMENT_BLEND_MODE_OPAQUE,
        XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND,
    };

    const std::vector<D3D_FEATURE_LEVEL> SupportedFeatureLevels = {
        D3D_FEATURE_LEVEL_12_1,
        D3D_FEATURE_LEVEL_12_0,
        D3D_FEATURE_LEVEL_11_1,
        D3D_FEATURE_LEVEL_11_0,
        D3D_FEATURE_LEVEL_10_1,
        D3D_FEATURE_LEVEL_10_0,
    };

    std::vector<const char*> AppendSceneLibRequiredExtensions(const std::vector<const char*>& extensions) {
        std::vector<const char*> mergedExtensions = extensions;
        for (auto extension : {
                 XR_KHR_D3D11_ENABLE_EXTENSION_NAME,
                 XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME,
                 XR_MSFT_UNBOUNDED_REFERENCE_SPACE_EXTENSION_NAME,
                 XR_MSFT_SECONDARY_VIEW_CONFIGURATION_PREVIEW_EXTENSION_NAME,
                 XR_MSFT_FIRST_PERSON_OBSERVER_PREVIEW_EXTENSION_NAME,
             }) {
            if (std::find(mergedExtensions.begin(), mergedExtensions.end(), extension) == mergedExtensions.end()) {
                mergedExtensions.push_back(extension);
            }
        }
        return mergedExtensions;
    }

    class ImplementXrApp : public XrApp {
    public:
        ImplementXrApp(const xr::NameVersion& appInfo, const std::vector<const char*>& desiredExtensions, bool singleThreadedD3D11Device);
        ~ImplementXrApp();

        void AddScene(std::unique_ptr<Scene> scene) override;
        const std::vector<std::unique_ptr<Scene>>& Scenes() const override;

        void Run() override;
        void Stop() override;

        ::SceneContext& SceneContext() const override {
            return *m_sceneContext;
        }

        ::ProjectionLayers& ProjectionLayers() override {
            return m_projectionLayerCollection;
        }

    private:

        xr::SessionHandle m_session;
        std::vector<DXGI_FORMAT> m_sessionSupportedColorSwapchainFormats;
        std::vector<DXGI_FORMAT> m_sessionSupportedDepthSwapchainFormats;
        std::vector<XrViewConfigurationType> m_enabledSecondaryViewConfigurationTypes;

        std::unique_ptr<::SceneContext> m_sceneContext;
        xr::SpaceHandle m_sceneSpace;

        // Make sure we declare projection layers after instance to ensure projection layers are destroyed before instance
        ::ProjectionLayers m_projectionLayerCollection = ::ProjectionLayers([this]() {
            auto ensureSwapchainFormatSupportedFunction = [&](DXGI_FORMAT format, bool isDepth) {
                if (isDepth) {
                    if (!xr::Contains(m_sessionSupportedDepthSwapchainFormats, format)) {
                        throw std::runtime_error(fmt::format("Unsupported depth swapchain format: {}", format).c_str());
                    }
                } else {
                    if (!xr::Contains(m_sessionSupportedColorSwapchainFormats, format)) {
                        throw std::runtime_error(fmt::format("Unsupported color swapchain format: {}", format).c_str());
                    }
                }
            };

            return std::make_unique<ProjectionLayer>(ensureSwapchainFormatSupportedFunction,
                                                     m_sessionSupportedColorSwapchainFormats[0],
                                                     m_sessionSupportedDepthSwapchainFormats[0],
                                                     PrimaryViewConfigurationType,
                                                     m_enabledSecondaryViewConfigurationTypes);
        });

        std::unordered_map<XrViewConfigurationType, xr::ViewConfigurationState> m_viewConfigState;

        std::mutex m_sceneMutex;
        std::vector<std::unique_ptr<Scene>> m_scenes;

        std::atomic<bool> m_sessionRunning{false};
        std::atomic<bool> m_appLoopRunning{false};

        std::thread m_renderThread;
        std::atomic<bool> m_renderThreadRunning{false};

        std::mutex m_frameReadyToRenderMutex;
        std::condition_variable m_frameReadyToRenderNotify;
        bool m_frameReadyToRender{false};
        FrameTime m_currentFrameTime;

    private:
        bool ProcessEvents();
        void StartRenderThreadIfNotRunning();
        void StopRenderThreadIfRunning();
        void UpdateFrame();
        void RenderFrame();
        void RenderViewConfiguration(const std::scoped_lock<std::mutex>& proofOfSceneLock,
                                     XrViewConfigurationType viewConfigurationType,
                                     CompositionLayers& layers);
        void SetSecondaryViewConfigurationActive(xr::ViewConfigurationState& secondaryViewConfigState, bool active);

        void FinalizeActionBindings(const std::scoped_lock<std::mutex>& proofOfSceneLock);
        void SyncActions(const std::scoped_lock<std::mutex>& proofOfSceneLock);

        void BeginSession();
        void EndSession();
    };

    ImplementXrApp::ImplementXrApp(const xr::NameVersion& appInfo,
                                   const std::vector<const char*>& desiredExtensions,
                                   bool singleThreadedD3D11Device) {

        // Create an instance using combined extensions of XrSceneLib and the application.
        // The extension context record those supported by the runtime and enabled by the instance.
        std::vector<const char*> combinedExtensions = AppendSceneLibRequiredExtensions(desiredExtensions);
        xr::ExtensionContext extensions = xr::CreateExtensionContext(combinedExtensions);
        xr::InstanceContext instance = xr::CreateInstanceContext(appInfo, {"XrSceneLib", 1}, extensions.EnabledExtensions);
        extensions.PopulateDispatchTable(instance.Handle);

        // For example, this sample currently requires D3D11 extension to be supported.
        if (!extensions.SupportsD3D11) {
            throw std::logic_error("This sample currently only supports D3D11.");
        }

        // Then get the active system with required form factor.
        // If no system is plugged in, wait until the device is plugged in.
        xr::SystemContext system = [&instance, &extensions] {
            std::optional<xr::SystemContext> systemOpt;
            while (!(systemOpt = xr::CreateSystemContext(instance.Handle,
                                                         extensions,
                                                         XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY,
                                                         SupportedViewConfigurationTypes,
                                                         SupportedEnvironmentBlendModes))) {
                sample::Trace("Waiting for system plugin ...");
                std::this_thread::sleep_for(500ms);
            }
            return systemOpt.value();
        }();

        auto [d3d11Binding, device, deviceContext] =
            sample::dx::CreateD3D11Binding(instance.Handle, system.Id, extensions, singleThreadedD3D11Device, SupportedFeatureLevels);

        XrSessionCreateInfo createInfo{XR_TYPE_SESSION_CREATE_INFO, &d3d11Binding, 0, system.Id};
        CHECK_XRCMD(xrCreateSession(instance.Handle, &createInfo, m_session.Put()));

        // In this sample, enable all system supported secondary view configurations
        m_enabledSecondaryViewConfigurationTypes = system.SupportedSecondaryViewConfigurationTypes;

        // Initialize XrViewConfigurationView and XrView buffers
        m_viewConfigState.emplace(PrimaryViewConfigurationType,
                                  xr::CreateViewConfigurationState(PrimaryViewConfigurationType, instance.Handle, system.Id));
        for (const auto& viewConfigurationType : m_enabledSecondaryViewConfigurationTypes) {
            m_viewConfigState.emplace(viewConfigurationType,
                                      xr::CreateViewConfigurationState(viewConfigurationType, instance.Handle, system.Id));
        }

        // Create main app space
        XrReferenceSpaceCreateInfo spaceCreateInfo{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
        spaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
        spaceCreateInfo.poseInReferenceSpace = xr::math::Pose::Identity();
        CHECK_XRCMD(xrCreateReferenceSpace(m_session.Get(), &spaceCreateInfo, m_sceneSpace.Put()));

        auto sessionSupportedSwapchainFormats = xr::EnumerateSwapchainFormats(m_session.Get());
        for (auto format : sessionSupportedSwapchainFormats) {
            if (xr::Contains(SupportedColorSwapchainFormats, format)) {
                m_sessionSupportedColorSwapchainFormats.push_back(static_cast<DXGI_FORMAT>(format));
            } else if (xr::Contains(SupportedDepthSwapchainFormats, format)) {
                m_sessionSupportedDepthSwapchainFormats.push_back(static_cast<DXGI_FORMAT>(format));
            }
        }

        Pbr::Resources pbrResources = sample::InitializePbrResources(device.get());

        m_sceneContext = std::make_unique<::SceneContext>(std::move(instance),
                                                          std::move(extensions),
                                                          std::move(system),
                                                          m_session.Get(),
                                                          PrimaryViewConfigurationType,
                                                          m_sceneSpace.Get(),
                                                          std::move(pbrResources),
                                                          device,
                                                          deviceContext);

        m_projectionLayerCollection.Resize(1, true /*forceReset*/);
    }

    ImplementXrApp::~ImplementXrApp() {
        StopRenderThreadIfRunning();

        {
            std::scoped_lock lock(m_sceneMutex);
            m_scenes.clear();
        }
    }

    void ImplementXrApp::AddScene(std::unique_ptr<Scene> scene) {
        if (!scene) {
            return; // Some scenes might skip creation due to extension unavailability.
        }

        std::scoped_lock lock(m_sceneMutex);
        assert(!m_appLoopRunning); // Cannot add scene after app loop is running

        m_scenes.push_back(std::move(scene));
    }

    const std::vector<std::unique_ptr<Scene>>& ImplementXrApp::Scenes() const {
        return m_scenes;
    }

    void ImplementXrApp::Run() {
        ::SetThreadDescription(::GetCurrentThread(), L"App Thread");

        m_appLoopRunning = true;
        {
            std::scoped_lock sceneLock(m_sceneMutex);
            FinalizeActionBindings(sceneLock);
        }

        while (m_appLoopRunning) {
            if (!ProcessEvents()) {
                break;
            }

            if (m_sessionRunning) {
                StartRenderThreadIfNotRunning();

                UpdateFrame();

                {
                    std::unique_lock lock(m_frameReadyToRenderMutex);
                    m_frameReadyToRender = true;
                }
                m_frameReadyToRenderNotify.notify_all();
            } else {
                std::this_thread::sleep_for(0.1s);
            }
        }
    }

    void ImplementXrApp::FinalizeActionBindings(const std::scoped_lock<std::mutex>& proofOfSceneLock) {
        std::vector<const xr::ActionContext*> actionContexts;
        for (const auto& scene : m_scenes) {
            actionContexts.push_back(&scene->ActionContext());
        }

        xr::AttachActionsToSession(SceneContext().Instance.Handle, SceneContext().Session, actionContexts);
    }

    void ImplementXrApp::SyncActions(const std::scoped_lock<std::mutex>& proofOfSceneLock) {
        std::vector<const xr::ActionContext*> actionContexts;
        for (const auto& scene : m_scenes) {
            if (scene->IsActive()) {
                actionContexts.push_back(&scene->ActionContext());
            }
        }
        xr::SyncActions(SceneContext().Session, actionContexts);
    }

    void ImplementXrApp::StartRenderThreadIfNotRunning() {
        bool alreadyRunning = false;
        if (m_renderThreadRunning.compare_exchange_strong(alreadyRunning, true)) {
            m_frameReadyToRender = false; // Always wait for xrWaitFrame before begin rendering frames.
            m_renderThread = std::thread([this]() {
                ::SetThreadDescription(::GetCurrentThread(), L"Render Thread");

                while (m_renderThreadRunning && m_sessionRunning) {
                    {
                        std::unique_lock lock(m_frameReadyToRenderMutex);
                        m_frameReadyToRenderNotify.wait(lock, [this] { return m_frameReadyToRender; });
                        m_frameReadyToRender = false;
                    }

                    if (!m_renderThreadRunning || !m_sessionRunning) {
                        break; // check again after waiting
                    }

                    RenderFrame();
                }
            });
        }
    }

    void ImplementXrApp::StopRenderThreadIfRunning() {
        bool alreadyRunning = true;
        if (m_renderThreadRunning.compare_exchange_strong(alreadyRunning, false)) {
            {
                // Notify "frameReadyToRender" with "renderThreadRunning = false" to exit render thread.
                std::unique_lock lock(m_frameReadyToRenderMutex);
                m_frameReadyToRender = true;
            }
            m_frameReadyToRenderNotify.notify_all();
            if (m_renderThread.joinable()) {
                m_renderThread.join();
            }
        }
    }

    void ImplementXrApp::Stop() {
        if (m_sessionRunning) {
            CHECK_XRCMD(xrRequestExitSession(SceneContext().Session));
        } else {
            m_appLoopRunning = false; // quit app message loop
        }
    }

    bool ImplementXrApp::ProcessEvents() {
        XrEventDataBuffer eventData;
        while (true) {
            eventData.type = XR_TYPE_EVENT_DATA_BUFFER;
            eventData.next = nullptr;
            XrResult res = xrPollEvent(SceneContext().Instance.Handle, &eventData);
            CHECK_XRCMD(res);
            if (res == XR_EVENT_UNAVAILABLE) {
                return true;
            }

            if (eventData.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
                auto* sessionStateChanged = xr::event_cast<XrEventDataSessionStateChanged>(&eventData);
                if (sessionStateChanged->session == SceneContext().Session) {
                    SceneContext().SessionState = sessionStateChanged->state;
                    switch (SceneContext().SessionState) {
                    case XR_SESSION_STATE_EXITING:
                        return false; // User's intended to quit
                    case XR_SESSION_STATE_LOSS_PENDING:
                        return false; // Runtime's intend to quit
                    case XR_SESSION_STATE_READY:
                        BeginSession();
                        break;
                    case XR_SESSION_STATE_STOPPING:
                        EndSession();
                        break;
                    }
                }
            }

            {
                std::scoped_lock lock(m_sceneMutex);
                for (const std::unique_ptr<Scene>& scene : m_scenes) {
                    scene->NotifyEvent(eventData);
                }
            }
        }
    }

    void ImplementXrApp::BeginSession() {
        XrSessionBeginInfo sessionBeginInfo{XR_TYPE_SESSION_BEGIN_INFO};
        sessionBeginInfo.primaryViewConfigurationType = PrimaryViewConfigurationType;

        XrSessionBeginSecondaryViewConfigurationInfoMSFT secondaryViewConfigInfo{
            XR_TYPE_SESSION_BEGIN_SECONDARY_VIEW_CONFIGURATION_INFO_MSFT};
        if (SceneContext().Extensions.SupportsSecondaryViewConfiguration && m_enabledSecondaryViewConfigurationTypes.size() > 0) {
            secondaryViewConfigInfo.viewConfigurationCount = (uint32_t)m_enabledSecondaryViewConfigurationTypes.size();
            secondaryViewConfigInfo.enabledViewConfigurationTypes = m_enabledSecondaryViewConfigurationTypes.data();
            xr::InsertExtensionStruct(sessionBeginInfo, secondaryViewConfigInfo);
        }

        CHECK_XRCMD(xrBeginSession(SceneContext().Session, &sessionBeginInfo));
        m_sessionRunning = true;
    }

    void ImplementXrApp::EndSession() {
        StopRenderThreadIfRunning();
        m_sessionRunning = false;
        CHECK_XRCMD(xrEndSession(SceneContext().Session));
    }

    void ImplementXrApp::UpdateFrame() {
        XrFrameWaitInfo waitFrameInfo{XR_TYPE_FRAME_WAIT_INFO};
        XrFrameState frameState{XR_TYPE_FRAME_STATE};
        std::vector<XrSecondaryViewConfigurationStateMSFT> secondaryViewConfigStateData;

        if (SceneContext().Extensions.SupportsSecondaryViewConfiguration && m_enabledSecondaryViewConfigurationTypes.size() > 0) {
            for (auto& secondaryViewConfigurtionType : m_enabledSecondaryViewConfigurationTypes) {
                secondaryViewConfigStateData.emplace_back(XrSecondaryViewConfigurationStateMSFT{
                    XR_TYPE_SECONDARY_VIEW_CONFIGURATION_STATE_MSFT, nullptr, secondaryViewConfigurtionType, XR_FALSE});
            }

            XrFrameSecondaryViewConfigurationsStateMSFT secondaryViewConfigStates{XR_TYPE_FRAME_SECONDARY_VIEW_CONFIGURATIONS_STATE_MSFT};
            secondaryViewConfigStates.viewConfigurationCount = (uint32_t)secondaryViewConfigStateData.size();
            secondaryViewConfigStates.states = secondaryViewConfigStateData.data();

            xr::InsertExtensionStruct(frameState, secondaryViewConfigStates);
        }

        CHECK_XRCMD(xrWaitFrame(SceneContext().Session, &waitFrameInfo, &frameState));

        if (SceneContext().Extensions.SupportsSecondaryViewConfiguration && secondaryViewConfigStateData.size() > 0) {
            for (const XrSecondaryViewConfigurationStateMSFT& state : secondaryViewConfigStateData) {
                SetSecondaryViewConfigurationActive(m_viewConfigState.at(state.viewConfigurationType), state.active);
            }
        }

        {
            std::scoped_lock sceneLock(m_sceneMutex);

            SyncActions(sceneLock);

            m_currentFrameTime.Update(frameState);

            for (auto& scene : m_scenes) {
                if (scene->IsActive()) {
                    scene->Update(m_currentFrameTime);
                }
            }
        }

        m_projectionLayerCollection.ForEachLayerWithLock([this](auto&& layer) {
            for (auto& [viewConfigType, states] : m_viewConfigState) {
                if (xr::IsPrimaryViewConfigurationType(viewConfigType) || states.Active) {
                    layer.PrepareRendering(SceneContext(), viewConfigType, states.ViewConfigViews);
                }
            }
        });
    }

    void ImplementXrApp::SetSecondaryViewConfigurationActive(xr::ViewConfigurationState& secondaryViewConfigState, bool active) {
        if (secondaryViewConfigState.Active != active) {
            secondaryViewConfigState.Active = active;

            // When a returned secondary view configuration is changed to active and recommended swapchain size is changed,
            // reset resources in layers related to this secondary view configuration.
            if (active) {
                std::vector<XrViewConfigurationView> newViewConfigViews = xr::EnumerateViewConfigurationViews(
                    SceneContext().Instance.Handle, SceneContext().System.Id, secondaryViewConfigState.Type);
                if (xr::IsRecommendedSwapchainSizeChanged(secondaryViewConfigState.ViewConfigViews, newViewConfigViews)) {
                    m_projectionLayerCollection.ForEachLayerWithLock(
                        [secondaryViewConfigType = secondaryViewConfigState.Type](auto&& layer) {
                            layer.Config(secondaryViewConfigType).ForceReset = true;
                        });
                }
            }
        }
    }

    void ImplementXrApp::RenderFrame() {
        XrFrameBeginInfo beginFrameDescription{XR_TYPE_FRAME_BEGIN_INFO};
        CHECK_XRCMD(xrBeginFrame(SceneContext().Session, &beginFrameDescription));

        XrFrameEndInfo endFrameInfo{XR_TYPE_FRAME_END_INFO};
        endFrameInfo.environmentBlendMode = SceneContext().PrimaryViewConfigurationBlendMode;
        endFrameInfo.displayTime = m_currentFrameTime.PredictedDisplayTime;

        // Secondary view config frame info need to have same lifetime as XrFrameEndInfo;
        XrFrameEndSecondaryViewConfigurationInfoMSFT frameEndSecondaryViewConfigInfo{
            XR_TYPE_FRAME_END_SECONDARY_VIEW_CONFIGURATION_INFO_MSFT};
        std::vector<XrSecondaryViewConfigurationLayerInfoMSFT> activeSecondaryViewConfigLayerInfos;

        // Chain secondary view configuration layers data to endFrameInfo
        if (SceneContext().Extensions.SupportsSecondaryViewConfiguration && m_enabledSecondaryViewConfigurationTypes.size() > 0) {
            for (auto& secondaryViewConfigType : m_enabledSecondaryViewConfigurationTypes) {
                auto& secondaryViewConfig = m_viewConfigState.at(secondaryViewConfigType);
                if (secondaryViewConfig.Active) {
                    activeSecondaryViewConfigLayerInfos.emplace_back(XrSecondaryViewConfigurationLayerInfoMSFT{
                        XR_TYPE_SECONDARY_VIEW_CONFIGURATION_LAYER_INFO_MSFT,
                        nullptr,
                        secondaryViewConfigType,
                        SceneContext().System.ViewProperties.at(secondaryViewConfigType).BlendMode});
                }
            }

            if (activeSecondaryViewConfigLayerInfos.size() > 0) {
                frameEndSecondaryViewConfigInfo.viewConfigurationCount = (uint32_t)activeSecondaryViewConfigLayerInfos.size();
                frameEndSecondaryViewConfigInfo.viewConfigurationLayersInfo = activeSecondaryViewConfigLayerInfos.data();
                xr::InsertExtensionStruct(endFrameInfo, frameEndSecondaryViewConfigInfo);
            }
        }

        // Prepare array of layer data for each active view configurations.
        std::vector<CompositionLayers> layersForAllViewConfigs(1 + activeSecondaryViewConfigLayerInfos.size());

        if (m_currentFrameTime.ShouldRender) {
            std::scoped_lock sceneLock(m_sceneMutex);
            CompositionLayers& primaryViewConfigLayers = layersForAllViewConfigs[0];
            RenderViewConfiguration(sceneLock, PrimaryViewConfigurationType, primaryViewConfigLayers);
            endFrameInfo.layerCount = primaryViewConfigLayers.LayerCount();
            endFrameInfo.layers = primaryViewConfigLayers.LayerData();

            if (SceneContext().Extensions.SupportsSecondaryViewConfiguration && activeSecondaryViewConfigLayerInfos.size() > 0) {
                for (size_t i = 0; i < activeSecondaryViewConfigLayerInfos.size(); i++) {
                    XrSecondaryViewConfigurationLayerInfoMSFT& secondaryViewConfigLayerInfo = activeSecondaryViewConfigLayerInfos.at(i);
                    CompositionLayers& secondaryViewConfigLayers = layersForAllViewConfigs.at(i + 1);
                    RenderViewConfiguration(sceneLock, secondaryViewConfigLayerInfo.viewConfigurationType, secondaryViewConfigLayers);
                    secondaryViewConfigLayerInfo.layerCount = secondaryViewConfigLayers.LayerCount();
                    secondaryViewConfigLayerInfo.layers = secondaryViewConfigLayers.LayerData();
                }
            }
        }

        CHECK_XRCMD(xrEndFrame(SceneContext().Session, &endFrameInfo));
    }

    void ImplementXrApp::RenderViewConfiguration(const std::scoped_lock<std::mutex>& proofOfSceneLock,
                                                 XrViewConfigurationType viewConfigurationType,
                                                 CompositionLayers& layers) {
        xr::ViewConfigurationState& viewConfigState = m_viewConfigState.at(viewConfigurationType);

        XrViewLocateInfo viewLocateInfo{XR_TYPE_VIEW_LOCATE_INFO};
        viewLocateInfo.viewConfigurationType = viewConfigurationType;
        viewLocateInfo.displayTime = m_currentFrameTime.PredictedDisplayTime;
        viewLocateInfo.space = SceneContext().SceneSpace;

        uint32_t viewCount = 0;
        XrViewState viewState{XR_TYPE_VIEW_STATE};

        std::vector<XrView>& views = viewConfigState.Views;
        CHECK_XRCMD(xrLocateViews(SceneContext().Session, &viewLocateInfo, &viewState, (uint32_t)views.size(), &viewCount, views.data()));
        assert(viewCount == views.size());

        if (xr::math::Pose::IsPoseValid(viewState)) {
            std::vector<std::shared_ptr<QuadLayerObject>> underlays, overlays;
            {
                // Collect all quad layers in active scenes
                for (const std::unique_ptr<Scene>& scene : m_scenes) {
                    if (!scene->IsActive()) {
                        continue;
                    }
                    for (const std::shared_ptr<QuadLayerObject>& quad : scene->GetQuadLayerObjects()) {
                        if (!quad->IsVisible()) {
                            continue;
                        }
                        if (quad->LayerGroup == LayerGrouping::Underlay) {
                            underlays.push_back(quad);
                        } else if (quad->LayerGroup == LayerGrouping::Overlay) {
                            overlays.push_back(quad);
                        }
                    }
                }
            }

            for (const std::shared_ptr<QuadLayerObject>& quad : underlays) {
                AppendQuadLayer(layers, quad.get());
            }

            m_projectionLayerCollection.ForEachLayerWithLock(
                [this, &layers, &views, viewConfigurationType](ProjectionLayer& projectionLayer) {
                    bool opaqueClearColor = (layers.LayerCount() == 0); // Only the first projection layer need opaque background
                    opaqueClearColor &= (SceneContext().PrimaryViewConfigurationBlendMode == XR_ENVIRONMENT_BLEND_MODE_OPAQUE);
                    projectionLayer.Config().ClearColor.v =
                        opaqueClearColor ? DirectX::XMColorSRGBToRGB(DirectX::Colors::CornflowerBlue) : DirectX::Colors::Transparent;
                    const bool shouldSubmitProjectionLayer = projectionLayer.Render(
                        SceneContext(), m_currentFrameTime, SceneContext().SceneSpace, views, m_scenes, viewConfigurationType);

                    // Create the multi projection layer
                    if (shouldSubmitProjectionLayer) {
                        AppendProjectionLayer(layers, &projectionLayer, viewConfigurationType);
                    }
                });

            for (const std::shared_ptr<QuadLayerObject>& quad : overlays) {
                AppendQuadLayer(layers, quad.get());
            }
        }
    }

} // namespace

std::unique_ptr<XrApp> CreateXrApp(const xr::NameVersion& appInfo,
                                   const std::vector<const char*>& desiredExtensions,
                                   bool singleThreadedD3D11Device) {
    return std::make_unique<ImplementXrApp>(appInfo, desiredExtensions, singleThreadedD3D11Device);
}

