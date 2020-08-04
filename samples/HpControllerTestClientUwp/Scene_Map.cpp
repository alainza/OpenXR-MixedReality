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
#include <XrSceneLib/PbrModelObject.h>
#include <XrSceneLib/Scene.h>

using namespace DirectX;
using namespace xr::math;
using namespace std::chrono;
using namespace std::chrono_literals;

namespace {

    struct __declspec(uuid("905a0fef-bc53-11df-8c49-001e4fc686da")) IBufferByteAccess : ::IUnknown
    {
        virtual HRESULT __stdcall Buffer(uint8_t** value) = 0;
    };

    const float fontScale = .10f;

    struct MapScene : public Scene {
        MapScene(SceneContext& sceneContext)
            : Scene(sceneContext) {

            xr::ActionSet& actionSet = ActionContext().CreateActionSet("move_map_actions", "Scene Move Map Actions");

            const std::vector<std::string> subactionPathBothHands = { "/user/hand/right", "/user/hand/left" };

            m_gripButtonAction = actionSet.CreateAction("grip_button_action", "Grip Button Action", XR_ACTION_TYPE_BOOLEAN_INPUT, subactionPathBothHands);
            m_gripAction = actionSet.CreateAction("grip_action", "Grip Action", XR_ACTION_TYPE_FLOAT_INPUT, subactionPathBothHands);
            m_gripPoseAction = actionSet.CreateAction("grip_pose", "Grip Pose", XR_ACTION_TYPE_POSE_INPUT, subactionPathBothHands);
            m_menuAction = actionSet.CreateAction("menu_action", "menu Action", XR_ACTION_TYPE_BOOLEAN_INPUT, subactionPathBothHands);
            m_selectAction = actionSet.CreateAction("select_action", "select Action", XR_ACTION_TYPE_FLOAT_INPUT, subactionPathBothHands);
            m_thumbstickAction = actionSet.CreateAction("thumbstick_action", "thumbstick Action", XR_ACTION_TYPE_VECTOR2F_INPUT, subactionPathBothHands);
            m_thumbstickButtonAction = actionSet.CreateAction("thumbstick_button_action", "Thumbstick Button Action", XR_ACTION_TYPE_BOOLEAN_INPUT, subactionPathBothHands);

#if (ENABLE_WMR_CONTROLLER)
            // https://www.khronos.org/registry/OpenXR/specs/1.0/html/xrspec.html#_microsoft_mixed_reality_motion_controller_profile
            ActionContext().SuggestInteractionProfileBindings("/interaction_profiles/microsoft/motion_controller",
                {
                    {m_gripButtonAction, "/user/hand/right/input/squeeze/click"},
                    {m_gripPoseAction, "/user/hand/right/input/grip/pose"},
                    {m_menuAction, "/user/hand/right/input/menu/click"},
                    {m_selectAction, "/user/hand/right/input/trigger/value"},
                    {m_thumbstickAction, "/user/hand/right/input/thumbstick"},

                    {m_gripButtonAction, "/user/hand/left/input/squeeze/click"},
                    {m_gripPoseAction, "/user/hand/left/input/grip/pose"},
                    {m_menuAction, "/user/hand/left/input/menu/click"},
                    {m_selectAction, "/user/hand/left/input/trigger/value"},
                    {m_thumbstickAction, "/user/hand/left/input/thumbstick"},
                });
#endif

            const std::vector<std::string> subactionPathLeftHand = { "/user/hand/left" };
            const std::vector<std::string> subactionPathRightHand = { "/user/hand/right" };

            m_aAction = actionSet.CreateAction("a_action", "a Action", XR_ACTION_TYPE_BOOLEAN_INPUT, subactionPathRightHand);
            m_bAction = actionSet.CreateAction("b_action", "b Action", XR_ACTION_TYPE_BOOLEAN_INPUT, subactionPathRightHand);
            m_xAction = actionSet.CreateAction("x_action", "x Action", XR_ACTION_TYPE_BOOLEAN_INPUT, subactionPathLeftHand);
            m_yAction = actionSet.CreateAction("y_action", "y Action", XR_ACTION_TYPE_BOOLEAN_INPUT, subactionPathLeftHand);

#if (ENABLE_HP_CONTROLLER)
            ActionContext().SuggestInteractionProfileBindings("/interaction_profiles/hp/motion_controller",
                {
                    {m_gripAction, "/user/hand/right/input/squeeze/value"},
                    {m_gripPoseAction, "/user/hand/right/input/grip/pose"},
                    {m_aAction, "/user/hand/right/input/a/click"},
                    {m_bAction, "/user/hand/right/input/b/click"},
                    {m_menuAction, "/user/hand/right/input/menu/click"},
                    {m_selectAction, "/user/hand/right/input/trigger/value"},
                    {m_thumbstickAction, "/user/hand/right/input/thumbstick"},
                    {m_thumbstickButtonAction, "/user/hand/right/input/thumbstick/click"},

                    {m_gripAction, "/user/hand/left/input/squeeze/value"},
                    {m_gripPoseAction, "/user/hand/left/input/grip/pose"},
                    {m_xAction, "/user/hand/left/input/x/click"},
                    {m_yAction, "/user/hand/left/input/y/click"},
                    {m_menuAction, "/user/hand/left/input/menu/click"},
                    {m_selectAction, "/user/hand/left/input/trigger/value"},
                    {m_thumbstickAction, "/user/hand/left/input/thumbstick"},
                    {m_thumbstickButtonAction, "/user/hand/left/input/thumbstick/click"},
                });
#endif
#if (ENABLE_OCULUS_CONTROLLER)
            ActionContext().SuggestInteractionProfileBindings("/interaction_profiles/oculus/touch_controller",
                {
                    {m_gripAction, "/user/hand/right/input/squeeze/value"},
                    {m_gripPoseAction, "/user/hand/right/input/grip/pose"},
                    {m_aAction, "/user/hand/right/input/a/click"},
                    {m_bAction, "/user/hand/right/input/b/click"},
                    {m_selectAction, "/user/hand/right/input/trigger/value"},
                    {m_thumbstickAction, "/user/hand/right/input/thumbstick"},

                    {m_gripAction, "/user/hand/left/input/squeeze/value"},
                    {m_gripPoseAction, "/user/hand/left/input/grip/pose"},
                    {m_xAction, "/user/hand/left/input/x/click"},
                    {m_yAction, "/user/hand/left/input/y/click"},
                    {m_selectAction, "/user/hand/left/input/trigger/value"},
                    {m_thumbstickAction, "/user/hand/left/input/thumbstick"},
                });
#endif

            // Use Squeeze for hands instead of Select as HL2 basically uses the same gesture for boths but adds
            // orientation constraints for Select, which does not allow placing cubes with various orientations
            if (sceneContext.Extensions.SupportsHandInteraction) {
                ActionContext().SuggestInteractionProfileBindings("/interaction_profiles/microsoft/hand_interaction",
                    {
                        {m_gripButtonAction, "/user/hand/right/input/squeeze/value"},
                        {m_gripPoseAction, "/user/hand/right/input/grip/pose"},

                        {m_gripButtonAction, "/user/hand/left/input/squeeze/value"},
                        {m_gripPoseAction, "/user/hand/left/input/grip/pose"},
                    });
            }

            XrReferenceSpaceCreateInfo referenceSpaceCreateInfo{ XR_TYPE_REFERENCE_SPACE_CREATE_INFO };
            referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
            referenceSpaceCreateInfo.poseInReferenceSpace = Pose::Identity();
            CHECK_XRCMD(xrCreateReferenceSpace(m_sceneContext.Session, &referenceSpaceCreateInfo, m_localSpace.Put()));

            if (m_sceneContext.Extensions.SupportsUnboundedSpace) {
                referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_UNBOUNDED_MSFT;
                CHECK_XRCMD(xrCreateReferenceSpace(m_sceneContext.Session, &referenceSpaceCreateInfo, m_unboundedSpace.Put()));
            }

            referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
            referenceSpaceCreateInfo.poseInReferenceSpace = Pose::Identity();
            CHECK_XRCMD(xrCreateReferenceSpace(m_sceneContext.Session, &referenceSpaceCreateInfo, m_viewSpace.Put()));

            auto createSpaceForHand = [&](XrPath hand, DirectX::XMVECTORF32 color)
            {
                XrActionSpaceCreateInfo spaceCreateInfo{ XR_TYPE_ACTION_SPACE_CREATE_INFO };
                spaceCreateInfo.poseInActionSpace = Pose::Identity();
                spaceCreateInfo.action = m_gripPoseAction;
                spaceCreateInfo.subactionPath = hand;

                xr::SpaceHandle spaceHandle;
                CHECK_XRCMD(xrCreateActionSpace(m_sceneContext.Session, &spaceCreateInfo, spaceHandle.Put()));

                // TODO: do that only if we are using controllers and not hands
                m_holograms.emplace_back(spaceHandle.Get(), AddSceneObject(CreateSphere(m_sceneContext.PbrResources, 0.05f, 20, Pbr::FromSRGB(color))));

                m_gripSpaces[hand] = std::move(spaceHandle);
            };

            createSpaceForHand(m_sceneContext.LeftHand, Colors::Cyan);
            createSpaceForHand(m_sceneContext.RightHand, Colors::Magenta);

            m_highlightMaterial = Pbr::Material::CreateFlat(m_sceneContext.PbrResources, Pbr::FromSRGB(Colors::Red));

            // Do not forget that quad has only one face and if you are looking at it the wrong way, it's invisible...
            // That's why we should always add cubes and not quads as static objects in scene, i.e.
            // reserve quads for always HMD Facing stuff only

            // Load and place A
            LoadLettersAsync();
        }

        void OnUpdate(const FrameTime& frameTime) override
        {

            {
                std::scoped_lock lock(m_hologramsMutex);
                for (auto& hologram : m_holograms) {
                    hologram.IsHighlighted = false;
                }
            }

            auto scaleHologram = [&](XrAction action, XrPath hand, int32_t idx)
            {
                if (idx != -1)
                {
                    XrActionStateFloat state{ XR_TYPE_ACTION_STATE_FLOAT};
                    XrActionStateGetInfo getInfo{ XR_TYPE_ACTION_STATE_GET_INFO };

                    getInfo.action = action;
                    getInfo.subactionPath = hand;
                    CHECK_XRCMD(xrGetActionStateFloat(m_sceneContext.Session, &getInfo, &state));
                    if (state.isActive)
                    {
                        auto sceneObject = m_holograms[idx].Object;
                        float newScale = fontScale * (1.0f - state.currentState * .9f);
                        sceneObject->Scale() = XrVector3f{ newScale, newScale, newScale };
                    }
                }
            };
            scaleHologram(m_selectAction, m_sceneContext.LeftHand, m_leftSelectIndex);
            scaleHologram(m_selectAction, m_sceneContext.RightHand, m_rightSelectIndex);
            scaleHologram(m_gripAction, m_sceneContext.LeftHand, m_leftGripIndex);
            scaleHologram(m_gripAction, m_sceneContext.RightHand, m_rightGripIndex);

            auto scale2DHologram = [&](XrAction action, XrPath hand, int32_t idx)
            {
                if (idx != -1)
                {
                    XrActionStateVector2f state{ XR_TYPE_ACTION_STATE_VECTOR2F };
                    XrActionStateGetInfo getInfo{ XR_TYPE_ACTION_STATE_GET_INFO };

                    getInfo.action = action;
                    getInfo.subactionPath = hand;
                    CHECK_XRCMD(xrGetActionStateVector2f(m_sceneContext.Session, &getInfo, &state));
                    if (state.isActive)
                    {
                        auto sceneObject = m_holograms[idx].Object;
                        float newScaleX = fontScale * (1.0f - state.currentState.x * .9f);
                        float newScaleY = fontScale * (1.0f - state.currentState.y * .9f);
                        sceneObject->Scale() = XrVector3f{ newScaleX, newScaleY, fontScale };
                    }
                }
            };
            scale2DHologram(m_thumbstickAction, m_sceneContext.LeftHand, m_leftThumbstickIndex);
            scale2DHologram(m_thumbstickAction, m_sceneContext.RightHand, m_rightThumbstickIndex);

            auto highlightIfPressed = [&](XrAction action, XrPath hand, int32_t idx)
            {
                if (idx != -1)
                {
                    XrActionStateBoolean state{ XR_TYPE_ACTION_STATE_BOOLEAN };
                    XrActionStateGetInfo getInfo{ XR_TYPE_ACTION_STATE_GET_INFO };

                    getInfo.action = action;
                    getInfo.subactionPath = hand;
                    CHECK_XRCMD(xrGetActionStateBoolean(m_sceneContext.Session, &getInfo, &state));
                    if (state.isActive && state.currentState) {
                        m_holograms[idx].IsHighlighted = true;
                    }
                }
            };

            highlightIfPressed(m_gripButtonAction, m_sceneContext.LeftHand, m_leftGripIndex);
            highlightIfPressed(m_thumbstickButtonAction, m_sceneContext.LeftHand, m_leftThumbstickIndex);
            highlightIfPressed(m_menuAction, m_sceneContext.LeftHand, m_leftMenuIndex);
            highlightIfPressed(m_xAction, m_sceneContext.LeftHand, m_leftXIndex);
            highlightIfPressed(m_yAction, m_sceneContext.LeftHand, m_leftYIndex);
            highlightIfPressed(m_gripButtonAction, m_sceneContext.RightHand, m_rightGripIndex);
            highlightIfPressed(m_thumbstickButtonAction, m_sceneContext.RightHand, m_rightThumbstickIndex);
            highlightIfPressed(m_menuAction, m_sceneContext.RightHand, m_rightMenuIndex);
            highlightIfPressed(m_aAction, m_sceneContext.RightHand, m_rightAIndex);
            highlightIfPressed(m_bAction, m_sceneContext.RightHand, m_rightBIndex);

            {
                std::scoped_lock lock(m_hologramsMutex);
                for (auto& hologram : m_holograms) {
                    auto model = hologram.Object->GetModel();
                    auto count = model->GetPrimitiveCount();
                    for (uint32_t i = 0; i < count; ++i)
                    {

                        auto& primitive = hologram.Object->GetModel()->GetPrimitive(i);

                        if (hologram.IsHighlighted)
                        {
                            primitive.SetAltMaterial(m_highlightMaterial);
                        }
                        else
                        {
                            primitive.ResetAltMaterial();
                        }
                        hologram.IsHighlighted = false;
                    }

                    UpdateHologramPlacement(hologram, frameTime.PredictedDisplayTime);
                }
            }
        }

    private:


    private:
        struct Hologram;

        void UpdateHologramPlacement(Hologram& hologram, XrTime time) {
            if (!hologram.Object) {
                return; // no visual to update.
            }

            XrSpaceLocation spaceLocation{ XR_TYPE_SPACE_LOCATION };
            CHECK_XRCMD(xrLocateSpace(hologram.Space, m_sceneContext.SceneSpace, time, &spaceLocation));

            if (Pose::IsPoseValid(spaceLocation)) {
                hologram.Object->SetVisible(true);
                if (hologram.Pose.has_value()) {
                    hologram.Object->Pose() = Pose::Multiply(hologram.Pose.value(), spaceLocation.pose);
                }
                else {
                    hologram.Object->Pose() = spaceLocation.pose;
                }

            }
            else {
                hologram.Object->SetVisible(false);
            }
        }

        private:
            winrt::Windows::Foundation::IAsyncAction LoadLetterAsync(const wchar_t* path, int32_t* leftIdx, int32_t* rightIdx, float dx)
            {
                winrt::Windows::Foundation::Uri uri(path);
                auto storage = co_await winrt::Windows::Storage::StorageFile::GetFileFromApplicationUriAsync(uri);
                auto buffer = co_await winrt::Windows::Storage::FileIO::ReadBufferAsync(storage);
                auto modelSize = static_cast<uint32_t>(buffer.Length());

                auto byteAccess = buffer.as<IBufferByteAccess>();
                uint8_t* bytes = nullptr;
                byteAccess->Buffer(&bytes);

                auto addHologram = [&](int32_t* idx, float dy)
                {
                    if (idx != nullptr)
                    {
                        // We could share more with a more complex highlighting mechanism...
                        auto model = Gltf::FromGltfBinary(m_sceneContext.PbrResources, bytes, modelSize);
                        auto modelObject = std::make_shared<PbrModelObject>(model);
                        auto sceneObject = AddSceneObject(modelObject);
                        sceneObject->Scale() = XrVector3f{ fontScale, fontScale, fontScale };
                        XrPosef pose;
                        pose.orientation = xr::math::Quaternion::Identity();
                        pose.position = { dx, dy, -1.0f };
                        {
                            std::scoped_lock lock(m_hologramsMutex);
                            *idx = static_cast<int32_t>(m_holograms.size());
                            m_holograms.emplace_back(m_viewSpace.Get(), sceneObject, pose).EnableCollisions = true;
                        }
                    }
                };
                addHologram(leftIdx, .2f);
                addHologram(rightIdx, -.2f);
            }

            winrt::Windows::Foundation::IAsyncAction LoadLettersAsync()
            {
                co_await LoadLetterAsync(L"ms-appx:///Assets/select.glb", &m_leftSelectIndex, &m_rightSelectIndex, -.65f);
                co_await LoadLetterAsync(L"ms-appx:///Assets/grasp.glb", &m_leftGripIndex, &m_rightGripIndex, -.35f);
                co_await LoadLetterAsync(L"ms-appx:///Assets/menu.glb", &m_leftMenuIndex, &m_rightMenuIndex, -0.05f);
                co_await LoadLetterAsync(L"ms-appx:///Assets/thumbstick.glb", &m_leftThumbstickIndex, &m_rightThumbstickIndex, .55f);
                co_await LoadLetterAsync(L"ms-appx:///Assets/a.glb", nullptr, &m_rightAIndex, .25f);
                co_await LoadLetterAsync(L"ms-appx:///Assets/b.glb", nullptr, &m_rightBIndex, .4f);
                co_await LoadLetterAsync(L"ms-appx:///Assets/x.glb", &m_leftXIndex, nullptr, .25f);
                co_await LoadLetterAsync(L"ms-appx:///Assets/y.glb", &m_leftYIndex, nullptr, .4f);
            }

    private:
        XrAction m_gripPoseAction{ XR_NULL_HANDLE };

        XrAction m_selectAction{ XR_NULL_HANDLE };
        XrAction m_gripButtonAction{ XR_NULL_HANDLE };
        XrAction m_gripAction{ XR_NULL_HANDLE };
        XrAction m_menuAction{ XR_NULL_HANDLE };
        XrAction m_thumbstickAction{ XR_NULL_HANDLE };
        XrAction m_thumbstickButtonAction{ XR_NULL_HANDLE };
        XrAction m_aAction{ XR_NULL_HANDLE };
        XrAction m_bAction{ XR_NULL_HANDLE };
        XrAction m_xAction{ XR_NULL_HANDLE };
        XrAction m_yAction{ XR_NULL_HANDLE };

        // These spaces are created wit xrCreateActionSpace() and therefore associated to actions
        // These actions are based on "pose" paths such as /user/hand/left/input/grip/pose
        std::unordered_map<XrPath, xr::SpaceHandle> m_gripSpaces;

        struct Hologram {
            Hologram() = default;
            Hologram(Hologram&) = delete;
            Hologram(Hologram&&) = default;

            Hologram(XrSpace space, std::shared_ptr<PbrModelObject> object, std::optional<XrPosef> pose = {}, bool enableCollisions = false)
                : Object(std::move(object))
                , Space(space)
                , Pose(pose)
                , EnableCollisions(enableCollisions) {
            }

            std::shared_ptr<PbrModelObject> Object;
            XrSpace Space = XR_NULL_HANDLE;
            std::optional<XrPosef> Pose = {};
            bool EnableCollisions = false;
            bool IsHighlighted = false;
        };

        std::mutex m_hologramsMutex;
        std::vector<Hologram> m_holograms;

        // Indices of holograms
        int32_t m_leftSelectIndex{ -1 };
        int32_t m_leftGripIndex{ -1 };
        int32_t m_leftMenuIndex{ -1 };
        int32_t m_leftXIndex{ -1 };
        int32_t m_leftYIndex{ -1 };
        int32_t m_rightSelectIndex{ -1 };
        int32_t m_rightGripIndex{ -1 };
        int32_t m_rightMenuIndex{ -1 };
        int32_t m_rightAIndex{ -1 };
        int32_t m_rightBIndex{ -1 };
        int32_t m_leftThumbstickIndex{ -1 };
        int32_t m_rightThumbstickIndex{ -1 };

        xr::SpaceHandle m_unboundedSpace;
        xr::SpaceHandle m_localSpace;
        xr::SpaceHandle m_viewSpace;

        struct AnchorSpace {
            xr::SpatialAnchorHandle Anchor;
            xr::SpaceHandle Space;
        };
        std::vector<AnchorSpace> m_anchorSpaces;

        std::shared_ptr<Pbr::Material> m_highlightMaterial;
    };
} // namespace

std::unique_ptr<Scene> TryCreateMapScene(SceneContext& sceneContext) {
    return std::make_unique<MapScene>(sceneContext);
}
