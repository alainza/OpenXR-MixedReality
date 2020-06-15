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


#define TEST_OCULUS_PROFILE 1

namespace {

    struct __declspec(uuid("905a0fef-bc53-11df-8c49-001e4fc686da")) IBufferByteAccess : ::IUnknown
    {
        virtual HRESULT __stdcall Buffer(uint8_t** value) = 0;
    };
    //
    // This sample shows different tracking behaviors across local, unbounded and anchor spaces.
    // The user can air-tap to create 3 cubes next to each other located precisely edge to edge when they are created.
    // The red cube is locked to the local space, the green to the unbounded space, and the blue to a newly created anchor space.
    // When the user walked tens of meters away and creating "three cubes" along the road,
    // she/he can observe the different behaviors of different type of spaces.
    // Typically each three-cubes will gradually tear apart due to different optimization of the underlying tracking techs.
    //
    struct MapScene : public Scene {
        MapScene(SceneContext& sceneContext)
            : Scene(sceneContext) {

            xr::ActionSet& actionSet = ActionContext().CreateActionSet("move_map_actions", "Scene Move Map Actions");

            const std::vector<std::string> subactionPathBothHands = { "/user/hand/right", "/user/hand/left" };

            m_gripAction = actionSet.CreateAction("grip_action", "Grip Action", XR_ACTION_TYPE_BOOLEAN_INPUT, subactionPathBothHands);
            m_gripPoseAction = actionSet.CreateAction("grip_pose", "Grip Pose", XR_ACTION_TYPE_POSE_INPUT, subactionPathBothHands);

            const std::vector<std::string> subactionPathLeftHand = { "/user/hand/left" };
            m_menuAction = actionSet.CreateAction("menu_action", "menu Action", XR_ACTION_TYPE_BOOLEAN_INPUT, subactionPathLeftHand);

#if (!TEST_OCULUS_PROFILE)

            // https://www.khronos.org/registry/OpenXR/specs/1.0/html/xrspec.html#_khronos_simple_controller_profile
            // /interaction_profiles/khr/simple_controller and /user/hand/right/input/squeeze/click generate an exception as squeeze is not supported in khr

            // https://www.khronos.org/registry/OpenXR/specs/1.0/html/xrspec.html#_microsoft_mixed_reality_motion_controller_profile
            ActionContext().SuggestInteractionProfileBindings("/interaction_profiles/microsoft/motion_controller",
                {
                    {m_gripAction, "/user/hand/right/input/squeeze/click"},
                    {m_gripAction, "/user/hand/left/input/squeeze/click"},
                    {m_gripPoseAction, "/user/hand/left/input/grip/pose"},
                    {m_gripPoseAction, "/user/hand/right/input/grip/pose"},
                    {m_menuAction,      "/user/hand/left/input/menu/click"},
                });

#else
            const std::vector<std::string> subactionPathRightHand = { "/user/hand/right" };

            m_aAction = actionSet.CreateAction("a_action", "a Action", XR_ACTION_TYPE_BOOLEAN_INPUT, subactionPathRightHand);
            m_bAction = actionSet.CreateAction("b_action", "b Action", XR_ACTION_TYPE_BOOLEAN_INPUT, subactionPathRightHand);
            m_xAction = actionSet.CreateAction("x_action", "x Action", XR_ACTION_TYPE_BOOLEAN_INPUT, subactionPathLeftHand);
            m_yAction = actionSet.CreateAction("y_action", "y Action", XR_ACTION_TYPE_BOOLEAN_INPUT, subactionPathLeftHand);

            // https://www.khronos.org/registry/OpenXR/specs/1.0/html/xrspec.html#_oculus_touch_controller_profile
            ActionContext().SuggestInteractionProfileBindings("/interaction_profiles/oculus/touch_controller",
                {
                    {m_gripAction,      "/user/hand/right/input/squeeze/value"},
                    {m_gripPoseAction,  "/user/hand/right/input/grip/pose"},
                    {m_aAction,         "/user/hand/right/input/a/click"},
                    {m_bAction,         "/user/hand/right/input/b/click"},

                    {m_gripAction,      "/user/hand/left/input/squeeze/value"},
                    {m_gripPoseAction,  "/user/hand/left/input/grip/pose"},
                    {m_xAction,         "/user/hand/left/input/x/click"},
                    {m_yAction,         "/user/hand/left/input/y/click"},
                    {m_menuAction,      "/user/hand/left/input/menu/click"},
                });

#endif

            // Use Squeeze for hands instead of Select as HL2 basically uses the same gesture for boths but adds
            // orientation constraints for Select, which does not allow placing cubes with various orientations
            if (sceneContext.Extensions.SupportsHandInteraction) {
                ActionContext().SuggestInteractionProfileBindings("/interaction_profiles/microsoft/hand_interaction",
                    {
                        {m_gripAction, "/user/hand/left/input/squeeze/value"},
                        {m_gripAction, "/user/hand/right/input/squeeze/value"},
                        {m_gripPoseAction, "/user/hand/left/input/grip/pose"},
                        {m_gripPoseAction, "/user/hand/right/input/grip/pose"},
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


            // Do not forget that quad has only one face and if you are looking at it the wrong way, it's invisible...
            // That's why we should always add cubes and not quads as static objects in scene, i.e.
            // reserve quads for always HMD Facing stuff only

            // Add the map
            XrPosef mapPose;
            mapPose.orientation = xr::math::Quaternion::RotationRollPitchYaw({ 1.57f, 0.0f, 0.0f });
            mapPose.position = { 0.0f, 0.0f, 0.0f };//  { 0.0f, -.5f, -1.0f };
            m_highlightMaterial = Pbr::Material::CreateFlat(m_sceneContext.PbrResources, Pbr::FromSRGB(Colors::Red));
            m_map = AddSceneObject(CreateCube(m_sceneContext.PbrResources, { .5f, .5f, .01f }, Pbr::FromSRGB(Colors::Green), 1, 0));
            m_holograms.emplace_back(m_unboundedSpace.Get(), m_map, mapPose).EnableCollisions = true;;

            // Load and place Rogue
            LoadRogueModelAsync();
        }

        void OnUpdate(const FrameTime& frameTime) override {

            if (m_requiresMapPlacement || m_requiresRoguePlacement)
            {
                // Let's try to find the position of map
                // based on user's looking direction
                XrSpaceLocation spaceLocation{ XR_TYPE_SPACE_LOCATION };
                if (XR_SUCCESS == xrLocateSpace(m_viewSpace.Get(), m_unboundedSpace.Get(), frameTime.PredictedDisplayTime, &spaceLocation))
                {
                    auto viewToUnbounded = LoadXrPose(spaceLocation.pose);
                    auto newPosition = XMVector3TransformCoord(XMVectorSet(0.0f, -.3f, -.8f, 1.0f), viewToUnbounded);
                    for (auto& hologram : m_holograms)
                    {
                        if (m_requiresMapPlacement && (hologram.Object == m_map))
                        {
                            XrPosef newPose = hologram.Pose.value();
                            StoreXrVector3(&newPose.position, newPosition);
                            hologram.Pose = newPose;
                            m_requiresMapPlacement = false;
                        }
                        else if (m_requiresRoguePlacement && (hologram.Object == m_rogue))
                        {
                            XrPosef newPose = hologram.Pose.value();
                            StoreXrVector3(&newPose.position, newPosition);
                            hologram.Pose = newPose;
                            m_requiresRoguePlacement = false;
                        }

                    }
                }
            }

            auto dumpBooleanActionState = [&](XrAction& action, const XrPath& subactionPath, const wchar_t* actionName)
            {
                XrActionStateBoolean state{ XR_TYPE_ACTION_STATE_BOOLEAN };
                XrActionStateGetInfo getInfo{ XR_TYPE_ACTION_STATE_GET_INFO };
                getInfo.action = action;
                getInfo.subactionPath = subactionPath;
                CHECK_XRCMD(xrGetActionStateBoolean(m_sceneContext.Session, &getInfo, &state));
                wchar_t debug[512];
                swprintf_s(debug, L"%s : active=%d  new=%d  was=%d\n", actionName, !!state.isActive, !!state.currentState, !!state.changedSinceLastSync);
                OutputDebugString(debug);
            };
#if (TEST_OCULUS_PROFILE)
            dumpBooleanActionState(m_aAction, m_sceneContext.RightHand, L"a");
            dumpBooleanActionState(m_bAction, m_sceneContext.RightHand, L"b");
            dumpBooleanActionState(m_xAction, m_sceneContext.LeftHand, L"x");
            dumpBooleanActionState(m_yAction, m_sceneContext.LeftHand, L"y");
#endif
            dumpBooleanActionState(m_menuAction, m_sceneContext.LeftHand, L"menu");
            dumpBooleanActionState(m_gripAction, m_sceneContext.RightHand, L"grip");
            dumpBooleanActionState(m_gripAction, m_sceneContext.LeftHand, L"grip");

            for (auto& [hand, space] : m_gripSpaces)
            {


                XrActionStateBoolean gripState{ XR_TYPE_ACTION_STATE_BOOLEAN };
                XrActionStateGetInfo getInfo{ XR_TYPE_ACTION_STATE_GET_INFO };
                getInfo.action = m_gripAction;

                getInfo.subactionPath = hand;
                CHECK_XRCMD(xrGetActionStateBoolean(m_sceneContext.Session, &getInfo, &gripState));

                // m_mapModelObject->GetModel()->GetPrimitive(0).UseAltMaterial(gripState.isActive && gripState.currentState);

                if (gripState.isActive && gripState.changedSinceLastSync && gripState.currentState)
                {
                    if (m_sceneContext.RightHand == hand)
                    {
                        m_requiresMapPlacement = true;
                    }
                    else
                    {
                        m_requiresRoguePlacement = true;
                    }
                }

                {
                    std::scoped_lock lock(m_hologramsMutex);
                    for (auto& hologram : m_holograms)
                    {
                        if (hologram.EnableCollisions)
                        {
                            // Try to compute the distance between map and controller / hand
                            // HL: (1) hl.Space + pose?
                            // Controller: space local variable
                            XrSpaceLocation spaceLocation{ XR_TYPE_SPACE_LOCATION };
                            bool collision = false;
                            if (XR_SUCCESS == xrLocateSpace(space.Get(), hologram.Space, frameTime.PredictedDisplayTime, &spaceLocation))
                            {
                                auto controllerToHLSpace = LoadXrPose(spaceLocation.pose);
                                auto hlToHLSpace = hologram.Pose.has_value() ? LoadXrPose(hologram.Pose.value()) : XMMatrixIdentity();
                                auto controllerToHL = controllerToHLSpace * XMMatrixInverse(nullptr, hlToHLSpace);
                                auto length = XMVectorGetX(XMVector3Length(XMVector3TransformCoord(XMVectorSet(0.0f, 0.0f, 0.0f, 1.0f), controllerToHL)));
                                collision = (length < .05f);

                                wchar_t debug[512];
                                swprintf_s(debug, L"Distance: %.4f\n", length);
                                OutputDebugString(debug);
                            }

                            hologram.IsHighlighted = collision;
                        }
                    }
                }
            }

            {
                std::scoped_lock lock(m_hologramsMutex);
                for (auto& hologram : m_holograms) {
                    auto& primitive = hologram.Object->GetModel()->GetPrimitive(0);

                    if (hologram.IsHighlighted)
                    {
                        primitive.SetAltMaterial(m_highlightMaterial);
                    }
                    else
                    {
                        primitive.ResetAltMaterial();
                    }
                    hologram.IsHighlighted = false;

                    UpdateHologramPlacement(hologram, frameTime.PredictedDisplayTime);
                }
            }
        }

    private:
        void PlaceCube(XrSpace space, XrTime time) {
            constexpr float cubeSize = 0.05f;
            constexpr XMFLOAT3 sideLength = { cubeSize, cubeSize, cubeSize };

            auto createHologram = [&](XrSpace baseSpace, const Pbr::RGBAColor& color, const XrVector3f& offset) {
                if (baseSpace == XR_NULL_HANDLE) {
                    return; // If extension is not supported, skip creating a hologram
                }

                XrSpaceLocation spaceLocation{ XR_TYPE_SPACE_LOCATION };
                CHECK_XRCMD(xrLocateSpace(space, baseSpace, time, &spaceLocation));

                Hologram hologram;
                hologram.Object = AddSceneObject(CreateCube(m_sceneContext.PbrResources, sideLength, color));
                hologram.Space = baseSpace;
                hologram.Pose = Pose::Multiply(Pose::Translation(offset), spaceLocation.pose);
                m_holograms.emplace_back(std::move(hologram));
            };

            createHologram(m_unboundedSpace.Get(), Pbr::FromSRGB(Colors::Green), { 0, 0, 0 });
        }

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

#if 0
        XrSpace CreateSpacialAnchorSpace(XrSpace space, XrTime time) {
            if (!m_sceneContext.Extensions.SupportsSpatialAnchor) {
                return XR_NULL_HANDLE; // cannot create hologram on spatial anchor.
            }

            AnchorSpace anchorSpace;
            XrSpaceLocation spaceLocation{ XR_TYPE_SPACE_LOCATION };
            CHECK_XRCMD(xrLocateSpace(space, m_sceneContext.SceneSpace, time, &spaceLocation));

            if (Pose::IsPoseValid(spaceLocation)) {
                XrSpatialAnchorCreateInfoMSFT createInfo{ XR_TYPE_SPATIAL_ANCHOR_CREATE_INFO_MSFT };
                createInfo.space = m_sceneContext.SceneSpace;
                createInfo.pose = spaceLocation.pose;
                createInfo.time = time;

                XrResult result = m_sceneContext.Extensions.xrCreateSpatialAnchorMSFT(
                    m_sceneContext.Session, &createInfo, anchorSpace.Anchor.Put(m_sceneContext.Extensions.xrDestroySpatialAnchorMSFT));
                if (result == XR_ERROR_CREATE_SPATIAL_ANCHOR_FAILED_MSFT) {
                    // Cannot create spatial anchor at this place
                    return XR_NULL_HANDLE;
                }
                else {
                    CHECK_XRRESULT(result, "xrCreateSpatialAnchorMSFT");
                }
            }

            if (anchorSpace.Anchor) {
                XrSpatialAnchorSpaceCreateInfoMSFT createInfo{ XR_TYPE_SPATIAL_ANCHOR_SPACE_CREATE_INFO_MSFT };
                createInfo.anchor = anchorSpace.Anchor.Get();
                createInfo.poseInAnchorSpace = Pose::Identity();
                CHECK_XRCMD(
                    m_sceneContext.Extensions.xrCreateSpatialAnchorSpaceMSFT(m_sceneContext.Session, &createInfo, anchorSpace.Space.Put()));
            }

            // Keep a copy of the handles to keep the anchor and space alive.
            return m_anchorSpaces.emplace_back(std::move(anchorSpace)).Space.Get();
        }
#endif

        private:
            winrt::Windows::Foundation::IAsyncAction LoadRogueModelAsync()
            {
                winrt::Windows::Foundation::Uri rogueUri(L"ms-appx:///Assets/Rogue.glb");
                auto rogueStorage = co_await winrt::Windows::Storage::StorageFile::GetFileFromApplicationUriAsync(rogueUri);
                auto buffer = co_await winrt::Windows::Storage::FileIO::ReadBufferAsync(rogueStorage);
                auto modelSize = static_cast<uint32_t>(buffer.Length());

                auto byteAccess = buffer.as<IBufferByteAccess>();
                uint8_t* bytes = nullptr;
                byteAccess->Buffer(&bytes);
                auto model = Gltf::FromGltfBinary(m_sceneContext.PbrResources, bytes, modelSize);

                m_rogue = AddSceneObject(std::make_shared<PbrModelObject>(model));
                m_rogue->Scale() = XrVector3f{ .01f, .01f, .01f };
                XrPosef roguePose;
                roguePose.orientation = xr::math::Quaternion::Identity();
                roguePose.position = { 0.0f, -.49f, -1.0f };
                {
                    std::scoped_lock lock(m_hologramsMutex);
                    m_holograms.emplace_back(m_unboundedSpace.Get(), m_rogue, roguePose).EnableCollisions = true;
                }
            }

    private:
        XrAction m_gripAction{ XR_NULL_HANDLE };
        XrAction m_gripPoseAction{ XR_NULL_HANDLE };
        XrAction m_menuAction{ XR_NULL_HANDLE };
#if (TEST_OCULUS_PROFILE)
        XrAction m_aAction{ XR_NULL_HANDLE };
        XrAction m_bAction{ XR_NULL_HANDLE };
        XrAction m_xAction{ XR_NULL_HANDLE };
        XrAction m_yAction{ XR_NULL_HANDLE };
#endif
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

        xr::SpaceHandle m_unboundedSpace;
        xr::SpaceHandle m_localSpace;
        xr::SpaceHandle m_viewSpace;

        struct AnchorSpace {
            xr::SpatialAnchorHandle Anchor;
            xr::SpaceHandle Space;
        };
        std::vector<AnchorSpace> m_anchorSpaces;

        bool m_requiresMapPlacement = true;
        std::shared_ptr<PbrModelObject> m_map;
        bool m_requiresRoguePlacement = false;
        std::shared_ptr<PbrModelObject> m_rogue;

        std::shared_ptr<Pbr::Material> m_highlightMaterial;
    };
} // namespace

std::unique_ptr<Scene> TryCreateMapScene(SceneContext& sceneContext) {
    return std::make_unique<MapScene>(sceneContext);
}
