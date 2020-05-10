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
#if 0
            xr::ActionSet& actionSet = ActionContext().CreateActionSet("three_cubes_scene_actions", "Three Cubes Scene Actions");

            const std::vector<std::string> subactionPathBothHands = { "/user/hand/right", "/user/hand/left" };

            m_selectAction = actionSet.CreateAction("select_action", "Select Action", XR_ACTION_TYPE_BOOLEAN_INPUT, subactionPathBothHands);
            m_aimPoseAction = actionSet.CreateAction("aim_pose", "Aim Pose", XR_ACTION_TYPE_POSE_INPUT, subactionPathBothHands);

            ActionContext().SuggestInteractionProfileBindings("/interaction_profiles/khr/simple_controller",
                {
                    {m_selectAction, "/user/hand/right/input/select/click"},
                    {m_selectAction, "/user/hand/left/input/select/click"},
                    {m_aimPoseAction, "/user/hand/left/input/aim/pose"},
                    {m_aimPoseAction, "/user/hand/right/input/aim/pose"},
                });

            // Use Squeeze for hands instead of Select as HL2 basically uses the same gesture for boths but adds
            // orientation constraints for Select, which does not allow placing cubes with various orientations
            if (sceneContext.Extensions.SupportsHandInteraction) {
                ActionContext().SuggestInteractionProfileBindings("/interaction_profiles/microsoft/hand_interaction",
                    {
                        {m_selectAction, "/user/hand/left/input/squeeze/value"},
                        {m_selectAction, "/user/hand/right/input/squeeze/value"},
                        {m_aimPoseAction, "/user/hand/left/input/aim/pose"},
                        {m_aimPoseAction, "/user/hand/right/input/aim/pose"},
                    });
            }

#endif

            XrReferenceSpaceCreateInfo referenceSpaceCreateInfo{ XR_TYPE_REFERENCE_SPACE_CREATE_INFO };
            referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
            referenceSpaceCreateInfo.poseInReferenceSpace = Pose::Identity();
            CHECK_XRCMD(xrCreateReferenceSpace(m_sceneContext.Session, &referenceSpaceCreateInfo, m_localSpace.Put()));

            if (m_sceneContext.Extensions.SupportsUnboundedSpace) {
                referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_UNBOUNDED_MSFT;
                CHECK_XRCMD(xrCreateReferenceSpace(m_sceneContext.Session, &referenceSpaceCreateInfo, m_unboundedSpace.Put()));
            }

#if 0
            XrActionSpaceCreateInfo spaceCreateInfo{ XR_TYPE_ACTION_SPACE_CREATE_INFO };
            spaceCreateInfo.poseInActionSpace = Pose::Identity();

            spaceCreateInfo.action = m_aimPoseAction;
            spaceCreateInfo.poseInActionSpace = Pose::Translation({ 0, 0, -0.2f });
            spaceCreateInfo.subactionPath = m_sceneContext.RightHand;
            CHECK_XRCMD(xrCreateActionSpace(m_sceneContext.Session, &spaceCreateInfo, m_rightAimSpace.Put()));
            spaceCreateInfo.subactionPath = m_sceneContext.LeftHand;
            CHECK_XRCMD(xrCreateActionSpace(m_sceneContext.Session, &spaceCreateInfo, m_leftAimSpace.Put()));

            m_holograms.emplace_back(m_rightAimSpace.Get(),
                AddSceneObject(CreateSphere(m_sceneContext.PbrResources, 0.05f, 20, Pbr::FromSRGB(Colors::Magenta))));

            m_holograms.emplace_back(m_leftAimSpace.Get(),
                AddSceneObject(CreateSphere(m_sceneContext.PbrResources, 0.05f, 20, Pbr::FromSRGB(Colors::Cyan))));
#endif


            // Do not forget that quad has only one face and if you are looking at it the wrong way, it's invisible...
            // TLDR: a flat cube might be better


#if 1
            XrPosef mapPose;
            mapPose.orientation = xr::math::Quaternion::RotationRollPitchYaw({ 1.57f, 0.0f, 0.0f });
            mapPose.position = { 0.0f, -.5f, -1.0f };
            m_map = AddSceneObject(CreateCube(m_sceneContext.PbrResources, { .5f, .5f, .01f }, Pbr::FromSRGB(Colors::Green), 1, 0));
            m_holograms.emplace_back(m_unboundedSpace.Get(), m_map, mapPose);
#endif

            LoadRogueModelAsync();
        }

        void OnUpdate(const FrameTime& frameTime) override {

#if 0
            XrActionStateBoolean selectState{ XR_TYPE_ACTION_STATE_BOOLEAN };
            XrActionStateGetInfo getInfo{ XR_TYPE_ACTION_STATE_GET_INFO };
            getInfo.action = m_selectAction;

            getInfo.subactionPath = m_sceneContext.RightHand;
            CHECK_XRCMD(xrGetActionStateBoolean(m_sceneContext.Session, &getInfo, &selectState));
            if (selectState.isActive && selectState.changedSinceLastSync && selectState.currentState) {
                PlaceThreeCubes(m_rightAimSpace.Get(), selectState.lastChangeTime);
            }

            getInfo.subactionPath = m_sceneContext.LeftHand;
            CHECK_XRCMD(xrGetActionStateBoolean(m_sceneContext.Session, &getInfo, &selectState));
            if (selectState.isActive && selectState.changedSinceLastSync && selectState.currentState) {
                PlaceThreeCubes(m_leftAimSpace.Get(), selectState.lastChangeTime);
            }

#endif

            {
                std::scoped_lock lock(m_hologramsMutex);
                for (auto& hologram : m_holograms) {
                    UpdateHologramPlacement(hologram, frameTime.PredictedDisplayTime);
                }
            }
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
                roguePose.orientation = xr::math::Quaternion::Identity(); //xr::math::Quaternion::RotationRollPitchYaw({ 1.57f, 0.0f, 0.0f });
                roguePose.position = { 0.0f, -.49f, -1.0f };
                {
                    std::scoped_lock lock(m_hologramsMutex);
                    m_holograms.emplace_back(m_unboundedSpace.Get(), m_rogue, roguePose);
                }
            }

    private:
        XrAction m_selectAction{ XR_NULL_HANDLE };
        XrAction m_aimPoseAction{ XR_NULL_HANDLE };

        xr::SpaceHandle m_rightAimSpace;
        xr::SpaceHandle m_leftAimSpace;

        struct Hologram {
            Hologram() = default;
            Hologram(Hologram&) = delete;
            Hologram(Hologram&&) = default;

            Hologram(XrSpace space, std::shared_ptr<SceneObject> object, std::optional<XrPosef> pose = {})
                : Object(std::move(object))
                , Space(space)
                , Pose(pose) {
            }

            std::shared_ptr<SceneObject> Object;
            XrSpace Space = XR_NULL_HANDLE;
            std::optional<XrPosef> Pose = {};
        };
        std::mutex m_hologramsMutex;
        std::vector<Hologram> m_holograms;

        xr::SpaceHandle m_unboundedSpace;
        xr::SpaceHandle m_localSpace;

        struct AnchorSpace {
            xr::SpatialAnchorHandle Anchor;
            xr::SpaceHandle Space;
        };
        std::vector<AnchorSpace> m_anchorSpaces;


        std::shared_ptr<SceneObject> m_map;
        std::shared_ptr<SceneObject> m_rogue;
    };
} // namespace

std::unique_ptr<Scene> TryCreateMapScene(SceneContext& sceneContext) {
    return std::make_unique<MapScene>(sceneContext);
}
