#include "core/arm_profile/ArmProfile.hpp"
#include "e-foc-protocol/contract/FocMotorWireContract.hpp"
#include "infra/stream/StreamManipulators.hpp"
#include "infra/syntax/Json.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <iterator>
#include <utility>

namespace arm
{
    namespace
    {
        constexpr const char* formatMarker{ "e-robot-arm profile" };
        constexpr std::array<const char*, 1> modeNames{ "torque" };
        constexpr std::array<const char*, 2> motionProfileNames{ "trapezoid", "s-curve" };

        std::array<std::pair<const char*, float*>, 6> ToolLimitFields(ToolLimits& limits)
        {
            return { { { "speedMPerS", &limits.speedMPerS }, { "accelerationMPerS2", &limits.accelerationMPerS2 }, { "jerkMPerS3", &limits.jerkMPerS3 },
                { "angularSpeedRadPerS", &limits.angularSpeedRadPerS }, { "angularAccelerationRadPerS2", &limits.angularAccelerationRadPerS2 },
                { "angularJerkRadPerS3", &limits.angularJerkRadPerS3 } } };
        }

        constexpr std::array<const char*, 4> currentNames{ "pid", "decoupledPid", "deadbeat", "slidingMode" };
        constexpr std::array<const char*, 4> speedNames{ "pid", "lqi", "adrc", "twoDof" };
        constexpr std::array<const char*, 5> positionNames{ "pid", "cascadeP", "lqr", "lqi", "twoDof" };
        constexpr std::array<const char*, 2> directionNames{ "positive", "negative" };
        constexpr std::array<const char*, gainCount> gainNames{ "currentBandwidth", "currentSwitchingGain", "currentBoundaryLayer",
            "currentTwoStepDeadbeat", "speedBandwidth", "speedErrorWeight", "speedIntegralWeight", "speedObserverBandwidthRatio",
            "speedReferenceTimeConstant", "positionBandwidth", "positionErrorWeight", "positionSpeedErrorWeight", "positionIntegralWeight",
            "positionReferenceTimeConstant" };

        constexpr std::array<const char*, 7> opwLengthNames{ "a1", "a2", "b", "c1", "c2", "c3", "c4" };
        constexpr std::array<const char*, 6> placementNames{ "xM", "yM", "zM", "rollRad", "pitchRad", "yawRad" };
        constexpr std::array<const char*, 3> centreNames{ "x", "y", "z" };
        constexpr std::array<const char*, 6> inertiaNames{ "xx", "yy", "zz", "xy", "xz", "yz" };

        constexpr std::array<double, 10> powersOfTen{ 1.0, 1.0e1, 1.0e2, 1.0e3, 1.0e4, 1.0e5, 1.0e6, 1.0e7, 1.0e8, 1.0e9 };

        using Path = infra::BoundedString::WithStorage<64>;
        using Value = infra::JsonValue;

        void Append(infra::BoundedString& target, infra::BoundedConstString text)
        {
            target.append(text.substr(0, std::min(text.size(), target.max_size() - target.size())));
        }

        Path Child(const Path& path, infra::BoundedConstString key)
        {
            Path child{ path };
            if (!child.empty())
                Append(child, ".");
            Append(child, key);
            return child;
        }

        Path Element(const Path& path, std::size_t index)
        {
            const std::array<char, 2> digit{ static_cast<char>('0' + index % 10), '\0' };
            Path element{ path };
            Append(element, "[");
            Append(element, digit.data());
            Append(element, "]");
            return element;
        }

        bool Mark(bool& seen, bool read)
        {
            seen = read;
            return read;
        }

        std::optional<double> Number(const Value& value)
        {
            if (const auto* integer = std::get_if<int32_t>(&value))
                return *integer;
            if (const auto* bigger = std::get_if<infra::JsonBiggerInt>(&value))
                return bigger->Negative() ? -static_cast<double>(bigger->Value()) : static_cast<double>(bigger->Value());
            if (const auto* real = std::get_if<infra::JsonFloat>(&value))
            {
                const auto magnitude = static_cast<double>(real->IntValue()) + static_cast<double>(real->NanoFractionalValue()) / 1.0e9;
                return real->Negative() ? -magnitude : magnitude;
            }

            return std::nullopt;
        }

        class Reader
        {
        public:
            explicit Reader(infra::BoundedString& problem)
                : problem(problem)
            {}

            std::optional<ArmProfile> Read(infra::BoundedConstString text)
            {
                problem.clear();
                ArmProfile profile{};
                profile.config.commandTimeoutPeriods = 3;
                if (!ReadRoot(Value{ infra::JsonObject{ text } }, profile))
                    return std::nullopt;
                return profile;
            }

        private:
            struct Root
            {
                bool format{ false };
                bool version{ false };
                bool name{ false };
                bool period{ false };
                bool missed{ false };
                bool joints{ false };
            };

            bool Fail(const Path& path, infra::BoundedConstString message)
            {
                problem.clear();
                Append(problem, path.empty() ? infra::BoundedConstString{ "profile" } : infra::BoundedConstString{ path });
                Append(problem, ": ");
                Append(problem, message);
                return false;
            }

            bool Missing(const Path& path, infra::BoundedConstString key)
            {
                return Fail(Child(path, key), "is missing");
            }

            template<class Member>
            bool ReadMembers(const Path& path, const Value& value, Member member)
            {
                const auto* found = std::get_if<infra::JsonObject>(&value);
                if (found == nullptr)
                    return Fail(path, "must be an object");

                auto object = *found;
                for (auto& item : object)
                    if (!member(Child(path, item.key.Raw()), item.key, item.value))
                        return false;

                if (object.Error())
                    return Fail(path, "is not valid JSON");
                return true;
            }

            bool Unknown(const Path& path)
            {
                return Fail(path, "is not a profile item");
            }

            bool ReadReal(const Path& path, const Value& value, float& result)
            {
                const auto number = Number(value);
                if (!number.has_value())
                    return Fail(path, "must be a number");
                result = static_cast<float>(*number);
                return true;
            }

            bool ReadNonNegative(const Path& path, const Value& value, float& result)
            {
                if (!ReadReal(path, value, result))
                    return false;
                if (!(result >= 0.0f))
                    return Fail(path, "must be 0 or above");
                return true;
            }

            bool ReadPositive(const Path& path, const Value& value, float& result)
            {
                if (!ReadReal(path, value, result))
                    return false;
                if (!(result > 0.0f))
                    return Fail(path, "must be above 0");
                return true;
            }

            bool ReadInteger(const Path& path, const Value& value, int32_t minimum, int32_t maximum, infra::BoundedConstString range, int32_t& result)
            {
                const auto* integer = std::get_if<int32_t>(&value);
                if (integer == nullptr || *integer < minimum || *integer > maximum)
                    return Fail(path, range);
                result = *integer;
                return true;
            }

            template<std::size_t N>
            bool ReadChoice(const Path& path, const Value& value, const std::array<const char*, N>& names, infra::BoundedConstString expected, std::size_t& result)
            {
                if (const auto* text = std::get_if<infra::JsonString>(&value))
                    for (std::size_t index = 0; index != N; ++index)
                        if (*text == names[index])
                        {
                            result = index;
                            return true;
                        }

                return Fail(path, expected);
            }

            bool ReadName(const Path& path, const Value& value, infra::BoundedString& name)
            {
                const auto* text = std::get_if<infra::JsonString>(&value);
                if (text == nullptr || text->empty() || text->size() > profileNameSize)
                    return Fail(path, "must be text of 1 to 48 characters");
                text->ToString(name);
                return true;
            }

            bool ReadRoot(const Value& value, ArmProfile& profile)
            {
                Root seen;
                const Path root;
                if (!ReadMembers(root, value, [this, &seen, &profile](const Path& path, infra::JsonString key, const Value& item)
                        {
                            return ReadRootItem(path, key, item, seen, profile);
                        }))
                    return false;

                if (!seen.format)
                    return Missing(root, "format");
                if (!seen.version)
                    return Missing(root, "version");
                if (!seen.name)
                    return Missing(root, "name");
                if (!seen.period)
                    return Missing(root, "controlPeriodMs");
                if (!seen.missed)
                    return Missing(root, "missedRequestsBeforeSilent");
                if (!seen.joints)
                    return Missing(root, "joints");
                const auto jerkLimited = std::ranges::all_of(profile.config.jerkLimits, [](const std::optional<foc::RadiansPerSecondCubed>& jerk)
                    {
                        return jerk.has_value();
                    });
                if (profile.config.motionProfile == MotionProfile::sCurve && !jerkLimited)
                    return Fail(Child(root, "motionProfile"), "s-curve needs a jerk limit on every joint");
                return true;
            }

            bool ReadRootItem(const Path& path, infra::JsonString key, const Value& value, Root& seen, ArmProfile& profile)
            {
                if (key == "format")
                    return ReadFormat(path, value, seen.format);
                if (key == "version")
                    return ReadVersion(path, value, seen.version);
                if (key == "name")
                    return Mark(seen.name, ReadName(path, value, profile.name));
                if (key == "controlPeriodMs")
                    return Mark(seen.period, ReadPeriod(path, value, profile.config));
                if (key == "missedRequestsBeforeSilent")
                    return Mark(seen.missed, ReadPeriods(path, value, profile.config.missedRequestsBeforeSilent));
                if (key == "commandTimeoutPeriods")
                    return ReadPeriods(path, value, profile.config.commandTimeoutPeriods);
                if (key == "driveWatchdogPeriods")
                    return ReadWatchdogPeriods(path, value, profile.config.driveWatchdogPeriods);
                if (key == "enableMode")
                    return ReadMode(path, value);
                if (key == "motionProfile")
                    return ReadMotionProfile(path, value, profile.config.motionProfile);
                if (key == "toolLimits")
                    return ReadToolLimits(path, value, profile.config.toolLimits);
                if (key == "teachPoints")
                    return ReadTeachPoints(path, value, profile.config);
                if (key == "joints")
                    return Mark(seen.joints, ReadJoints(path, value, profile.config));
                if (key == "geometry")
                    return ReadGeometry(path, value, profile.config.geometry);
                if (key == "massProperties")
                    return ReadMass(path, value, profile.config.mass);
                if (key == "meshes")
                    return ReadMeshes(path, value, profile.meshes);
                if (key == "gravityPd")
                    return ReadTuning(path, value, profile.config.gravityPd);
                if (key == "computedTorque")
                    return ReadTuning(path, value, profile.config.computedTorque);
                if (key == "gravityPid")
                    return ReadTuning(path, value, profile.config.gravityPid);
                if (key == "jointImpedance")
                    return ReadImpedance(path, value, profile.config.jointImpedance);
                if (key == "collisionObserverGainPerS")
                    return ReadBounded(path, value, 1000.0f, "must be above 0 and at most 1000", profile.config.collisionObserverGain);
                if (key == "safety")
                    return ReadSafety(path, value, profile.config.workspace);
                return Unknown(path);
            }

            bool ReadTeachPoints(const Path& path, const Value& value, ArmConfig& config)
            {
                std::size_t count{ 0 };
                if (!ReadList(path, value, 0, maxTeachPoints, "must be a list of at most sixteen points", count,
                        [this, &config](const Path& item, const Value& element, std::size_t index)
                        {
                            return ReadTeachPoint(item, element, config.teachPoints[index]);
                        }))
                    return false;

                config.teachPointCount = static_cast<uint8_t>(count);
                for (std::size_t point = 1; point < count; ++point)
                    for (std::size_t earlier = 0; earlier != point; ++earlier)
                        if (config.teachPoints[point].name == config.teachPoints[earlier].name)
                            return Fail(path, "names must be unique");
                return true;
            }

            bool ReadTeachPoint(const Path& path, const Value& value, TeachPoint& point)
            {
                bool named{ false };
                bool placed{ false };
                const auto read = ReadMembers(path, value, [this, &point, &named, &placed](const Path& item, infra::JsonString key, const Value& member)
                    {
                        if (key == "name")
                            return Mark(named, ReadPointName(item, member, point.name));
                        if (key == "jointsRad")
                        {
                            std::size_t count{ 0 };
                            return Mark(placed, ReadList(item, member, jointCount, jointCount, "must be a list of six angles", count,
                                                    [this, &point](const Path& angle, const Value& element, std::size_t index)
                                                    {
                                                        float radians{ 0.0f };
                                                        return ReadReal(angle, element, radians) && Store(point.joints[index], foc::Radians{ radians });
                                                    }));
                        }
                        return Unknown(item);
                    });
                if (!read)
                    return false;
                if (!named || !placed)
                    return Fail(path, "needs a name and jointsRad");
                return true;
            }

            bool ReadPointName(const Path& path, const Value& value, infra::BoundedString& name)
            {
                const auto* text = std::get_if<infra::JsonString>(&value);
                if (text == nullptr || text->empty() || text->size() > teachPointNameSize)
                    return Fail(path, "must be text of 1 to 16 characters");
                text->ToString(name);
                return true;
            }

            bool ReadToolLimits(const Path& path, const Value& value, ToolLimits& limits)
            {
                return ReadMembers(path, value, [this, &limits](const Path& item, infra::JsonString key, const Value& member)
                    {
                        if (key == "manipulabilityMin")
                            return ReadManipulability(item, member, limits.manipulabilityMin);
                        for (const auto& [name, field] : ToolLimitFields(limits))
                            if (key == name)
                                return ReadPositive(item, member, *field);
                        return Unknown(item);
                    });
            }

            bool ReadManipulability(const Path& path, const Value& value, float& result)
            {
                float number{ 0.0f };
                if (!ReadReal(path, value, number))
                    return false;
                if (!(number >= 0.0f && number <= 1.0f))
                    return Fail(path, "must be 0 or above and at most 1");
                result = number;
                return true;
            }

            bool ReadTuning(const Path& path, const Value& value, BandwidthTuning& tuning)
            {
                return ReadMembers(path, value, [this, &tuning](const Path& item, infra::JsonString key, const Value& member)
                    {
                        if (key == "bandwidthRadPerS")
                            return ReadBounded(item, member, 500.0f, "must be above 0 and at most 500", tuning.bandwidth);
                        if (key == "dampingRatio")
                            return ReadBounded(item, member, 5.0f, "must be above 0 and at most 5", tuning.dampingRatio);
                        return Unknown(item);
                    });
            }

            bool ReadImpedance(const Path& path, const Value& value, JointImpedanceTuning& tuning)
            {
                return ReadMembers(path, value, [this, &tuning](const Path& item, infra::JsonString key, const Value& member)
                    {
                        if (key == "stiffnessNmPerRad")
                        {
                            JointStiffnesses stiffness{};
                            if (!ReadSix(item, member, stiffness, [this](const Path& element, const Value& entry, float& result)
                                    {
                                        return ReadPositive(element, entry, result);
                                    }))
                                return false;
                            tuning.stiffness = stiffness;
                            return true;
                        }
                        if (key == "dampingRatio")
                            return ReadBounded(item, member, 5.0f, "must be above 0 and at most 5", tuning.dampingRatio);
                        return Unknown(item);
                    });
            }

            bool ReadBounded(const Path& path, const Value& value, float maximum, infra::BoundedConstString range, float& result)
            {
                float number{ 0.0f };
                if (!ReadReal(path, value, number))
                    return false;
                if (!(number > 0.0f && number <= maximum))
                    return Fail(path, range);
                result = number;
                return true;
            }

            bool ReadMeshes(const Path& path, const Value& value, std::array<infra::BoundedString::WithStorage<meshPathSize>, meshCount>& meshes)
            {
                return ReadMembers(path, value, [this, &meshes](const Path& item, infra::JsonString key, const Value& member)
                    {
                        for (std::size_t index = 0; index != meshCount; ++index)
                            if (key == meshNames[index])
                                return ReadMesh(item, member, meshes[index]);
                        return Unknown(item);
                    });
            }

            bool ReadMesh(const Path& path, const Value& value, infra::BoundedString& mesh)
            {
                const auto* text = std::get_if<infra::JsonString>(&value);
                if (text == nullptr || text->empty() || text->size() > meshPathSize)
                    return Fail(path, "must be a path of 1 to 96 characters");
                text->ToString(mesh);
                if (!EndsWith(mesh, ".gltf") && !EndsWith(mesh, ".glb"))
                    return Fail(path, "must name a .gltf or .glb file");
                return true;
            }

            static bool EndsWith(infra::BoundedConstString text, infra::BoundedConstString suffix)
            {
                return text.size() >= suffix.size() && text.substr(text.size() - suffix.size()) == suffix;
            }

            bool ReadMass(const Path& path, const Value& value, ArmMass& mass)
            {
                mass = ArmMass{};
                bool links{ false };
                if (!ReadMembers(path, value, [this, &mass, &links](const Path& item, infra::JsonString key, const Value& member)
                        {
                            if (key == "links")
                                return Mark(links, ReadSix(item, member, mass.links, [this](const Path& element, const Value& entry, MassProperties& properties)
                                                       {
                                                           return ReadMassProperties(element, entry, properties);
                                                       }));
                            if (key == "payload")
                                return ReadMassProperties(item, member, mass.payload);
                            return Unknown(item);
                        }))
                    return false;

                if (!links)
                    return Missing(path, "links");
                return true;
            }

            bool ReadMassProperties(const Path& path, const Value& value, MassProperties& properties)
            {
                bool massSeen{ false };
                if (!ReadMembers(path, value, [this, &properties, &massSeen](const Path& item, infra::JsonString key, const Value& member)
                        {
                            if (key == "massKg")
                                return Mark(massSeen, ReadNonNegative(item, member, properties.mass));
                            if (key == "centreM")
                                return ReadNamed(item, member, centreNames, properties.centre);
                            if (key == "inertiaKgM2")
                                return ReadNamed(item, member, inertiaNames, properties.inertia) && CheckInertia(item, properties.inertia);
                            return Unknown(item);
                        }))
                    return false;

                if (!massSeen)
                    return Missing(path, "massKg");
                return true;
            }

            template<std::size_t N>
            bool ReadNamed(const Path& path, const Value& value, const std::array<const char*, N>& names, std::array<float, N>& values)
            {
                return ReadMembers(path, value, [this, &names, &values](const Path& item, infra::JsonString key, const Value& member)
                    {
                        for (std::size_t index = 0; index != N; ++index)
                            if (key == names[index])
                                return ReadReal(item, member, values[index]);
                        return Unknown(item);
                    });
            }

            bool CheckInertia(const Path& path, const std::array<float, 6>& inertia)
            {
                const auto xx = inertia[0];
                const auto yy = inertia[1];
                const auto zz = inertia[2];
                if (!(xx >= 0.0f && yy >= 0.0f && zz >= 0.0f))
                    return Fail(path, "moments must be 0 or above");
                if (xx + yy < zz || yy + zz < xx || zz + xx < yy)
                    return Fail(path, "each sum of two moments must be at least the third");
                return true;
            }

            bool ReadGeometry(const Path& path, const Value& value, ArmGeometry& geometry)
            {
                geometry = ArmGeometry{};
                bool lengths{ false };
                if (!ReadMembers(path, value, [this, &geometry, &lengths](const Path& item, infra::JsonString key, const Value& member)
                        {
                            if (key == "opwMetres")
                                return Mark(lengths, ReadLengths(item, member, geometry.opw));
                            if (key == "jointSigns")
                                return ReadSigns(item, member, geometry.opw.signs);
                            if (key == "jointOffsetsRad")
                                return ReadOffsets(item, member, geometry.opw.offsets);
                            if (key == "tool")
                                return ReadPlacement(item, member, geometry.tool);
                            if (key == "base")
                                return ReadPlacement(item, member, geometry.base);
                            return Unknown(item);
                        }))
                    return false;

                if (!lengths)
                    return Missing(path, "opwMetres");
                return true;
            }

            bool ReadLengths(const Path& path, const Value& value, kinematics::OpwParameters<float>& opw)
            {
                std::array<bool, opwLengthNames.size()> seen{};
                std::array<float*, opwLengthNames.size()> lengths{ &opw.a1, &opw.a2, &opw.b, &opw.c1, &opw.c2, &opw.c3, &opw.c4 };
                if (!ReadMembers(path, value, [this, &seen, &lengths](const Path& item, infra::JsonString key, const Value& member)
                        {
                            for (std::size_t index = 0; index != opwLengthNames.size(); ++index)
                                if (key == opwLengthNames[index])
                                    return Mark(seen[index], ReadReal(item, member, *lengths[index]));
                            return Unknown(item);
                        }))
                    return false;

                for (std::size_t index = 0; index != opwLengthNames.size(); ++index)
                    if (!seen[index])
                        return Missing(path, opwLengthNames[index]);
                if (!(opw.c2 > 0.0f))
                    return Fail(Child(path, "c2"), "must be above 0");
                if (!(opw.c3 > 0.0f))
                    return Fail(Child(path, "c3"), "must be above 0");
                return true;
            }

            template<class Item, class ReadItem>
            bool ReadSix(const Path& path, const Value& value, std::array<Item, 6>& result, ReadItem readItem)
            {
                const auto* found = std::get_if<infra::JsonArray>(&value);
                if (found == nullptr)
                    return Fail(path, "must be a list of six");

                auto list = *found;
                std::size_t count{ 0 };
                for (const auto& element : list)
                {
                    if (count == result.size())
                        return Fail(path, "must be a list of six");
                    if (!readItem(Element(path, count), element, result[count]))
                        return false;
                    ++count;
                }

                if (list.Error())
                    return Fail(path, "is not valid JSON");
                if (count != result.size())
                    return Fail(path, "must be a list of six");
                return true;
            }

            bool ReadSigns(const Path& path, const Value& value, std::array<int8_t, 6>& signs)
            {
                return ReadSix(path, value, signs, [this](const Path& item, const Value& element, int8_t& sign)
                    {
                        const auto* integer = std::get_if<int32_t>(&element);
                        if (integer == nullptr || (*integer != 1 && *integer != -1))
                            return Fail(item, "must be 1 or -1");
                        sign = static_cast<int8_t>(*integer);
                        return true;
                    });
            }

            bool ReadOffsets(const Path& path, const Value& value, std::array<float, 6>& offsets)
            {
                return ReadSix(path, value, offsets, [this](const Path& item, const Value& element, float& offset)
                    {
                        return ReadReal(item, element, offset);
                    });
            }

            bool ReadPlacement(const Path& path, const Value& value, Placement& placement)
            {
                std::array<float*, placementNames.size()> fields{ &placement.x, &placement.y, &placement.z, &placement.roll, &placement.pitch, &placement.yaw };
                return ReadMembers(path, value, [this, &fields](const Path& item, infra::JsonString key, const Value& member)
                    {
                        for (std::size_t index = 0; index != placementNames.size(); ++index)
                            if (key == placementNames[index])
                                return ReadReal(item, member, *fields[index]);
                        return Unknown(item);
                    });
            }

            template<class ReadItem>
            bool ReadList(const Path& path, const Value& value, std::size_t minimum, std::size_t maximum, infra::BoundedConstString shape, std::size_t& count,
                ReadItem readItem)
            {
                const auto* found = std::get_if<infra::JsonArray>(&value);
                if (found == nullptr)
                    return Fail(path, shape);

                auto list = *found;
                count = 0;
                for (const auto& element : list)
                {
                    if (count == maximum)
                        return Fail(path, shape);
                    if (!readItem(Element(path, count), element, count))
                        return false;
                    ++count;
                }

                if (list.Error())
                    return Fail(path, "is not valid JSON");
                if (count < minimum)
                    return Fail(path, shape);
                return true;
            }

            bool ReadSafety(const Path& path, const Value& value, WorkspaceConfig& workspace)
            {
                return ReadMembers(path, value, [this, &workspace](const Path& item, infra::JsonString key, const Value& member)
                    {
                        return ReadSafetyItem(item, key, member, workspace);
                    });
            }

            bool ReadSafetyItem(const Path& path, infra::JsonString key, const Value& value, WorkspaceConfig& workspace)
            {
                std::size_t count{ 0 };
                if (key == "workspaceM")
                {
                    WorkspaceBox box{};
                    return ReadBox(path, value, box) && Store(workspace.workspace, std::optional{ box });
                }
                if (key == "forbiddenZonesM")
                    return ReadList(path, value, 0, maxForbiddenZones, "must be a list of at most eight boxes", count,
                               [this, &workspace](const Path& item, const Value& element, std::size_t index)
                               {
                                   return ReadBox(item, element, workspace.forbiddenZones[index]);
                               }) &&
                           Store(workspace.forbiddenZoneCount, static_cast<uint8_t>(count));
                if (key == "linkRadiiM")
                    return ReadList(path, value, workspaceCapsules, workspaceCapsules, "must be a list of seven radii", count,
                        [this, &workspace](const Path& item, const Value& element, std::size_t index)
                        {
                            return ReadPositive(item, element, workspace.linkRadii[index]);
                        });
                if (key == "selfCollisionMarginM")
                    return ReadMargin(path, value, workspace.selfCollisionMargin);
                if (key == "allowedLinkPairs")
                    return ReadList(path, value, 0, maxAllowedLinkPairs, "must be a list of at most sixteen pairs", count,
                               [this, &workspace](const Path& item, const Value& element, std::size_t index)
                               {
                                   return ReadLinkPair(item, element, workspace.allowedLinkPairs[index]);
                               }) &&
                           Store(workspace.allowedLinkPairCount, static_cast<uint8_t>(count));
                return Unknown(path);
            }

            bool ReadBox(const Path& path, const Value& value, WorkspaceBox& box)
            {
                static constexpr std::array<const char*, 3> axes{ "x", "y", "z" };
                std::array<float, 3> low{};
                std::array<float, 3> high{};
                if (!ReadMembers(path, value, [this, &low, &high](const Path& item, infra::JsonString key, const Value& member)
                        {
                            if (key == "min")
                                return ReadNamed(item, member, axes, low);
                            if (key == "max")
                                return ReadNamed(item, member, axes, high);
                            return Unknown(item);
                        }))
                    return false;

                for (std::size_t axis = 0; axis != 3; ++axis)
                    if (!(low[axis] < high[axis]))
                        return Fail(path, "min must be below max on every axis");
                box = { math::Vector3<float>{ low[0], low[1], low[2] }, math::Vector3<float>{ high[0], high[1], high[2] } };
                return true;
            }

            bool ReadMargin(const Path& path, const Value& value, float& margin)
            {
                float number{ 0.0f };
                if (!ReadReal(path, value, number))
                    return false;
                if (!(number >= 0.0f && number <= 0.5f))
                    return Fail(path, "must be from 0 to 0.5");
                margin = number;
                return true;
            }

            bool ReadLinkPair(const Path& path, const Value& value, LinkPair& pair)
            {
                std::array<int32_t, 2> links{};
                std::size_t count{ 0 };
                if (!ReadList(path, value, 2, 2, "must be a pair of links", count, [this, &links](const Path& item, const Value& element, std::size_t index)
                        {
                            return ReadInteger(item, element, 1, static_cast<int32_t>(workspaceCapsules), "must be a link from 1 to 7", links[index]);
                        }))
                    return false;
                if (links[0] - links[1] >= -1 && links[0] - links[1] <= 1)
                    return Fail(path, "must name two links that are not adjacent");
                pair = { static_cast<uint8_t>(links[0] - 1), static_cast<uint8_t>(links[1] - 1) };
                return true;
            }

            bool ReadFormat(const Path& path, const Value& value, bool& seen)
            {
                const auto* text = std::get_if<infra::JsonString>(&value);
                if (text == nullptr || !(*text == formatMarker))
                    return Fail(path, "must be \"e-robot-arm profile\"");
                seen = true;
                return true;
            }

            bool ReadVersion(const Path& path, const Value& value, bool& seen)
            {
                int32_t version{ 0 };
                if (!ReadInteger(path, value, 1, profileVersion, "is not a version this reader knows", version))
                    return false;
                seen = true;
                return true;
            }

            bool ReadPeriod(const Path& path, const Value& value, ArmConfig& config)
            {
                int32_t milliseconds{ 0 };
                if (!ReadInteger(path, value, 1, 1000, "must be a whole number from 1 to 1000", milliseconds))
                    return false;
                config.controlPeriod = std::chrono::milliseconds{ milliseconds };
                return true;
            }

            bool ReadPeriods(const Path& path, const Value& value, uint8_t& result)
            {
                int32_t periods{ 0 };
                if (!ReadInteger(path, value, 1, 255, "must be a whole number from 1 to 255", periods))
                    return false;
                result = static_cast<uint8_t>(periods);
                return true;
            }

            bool ReadWatchdogPeriods(const Path& path, const Value& value, uint8_t& result)
            {
                int32_t periods{ 0 };
                if (!ReadInteger(path, value, 2, 255, "must be a whole number from 2 to 255", periods))
                    return false;
                result = static_cast<uint8_t>(periods);
                return true;
            }

            bool ReadMode(const Path& path, const Value& value)
            {
                std::size_t mode{ 0 };
                return ReadChoice(path, value, modeNames, "must be torque: the arm runs its joints in torque mode only", mode);
            }

            bool ReadMotionProfile(const Path& path, const Value& value, MotionProfile& profile)
            {
                std::size_t chosen{ 0 };
                if (!ReadChoice(path, value, motionProfileNames, "must be trapezoid or s-curve", chosen))
                    return false;
                profile = static_cast<MotionProfile>(chosen);
                return true;
            }

            bool ReadJoints(const Path& path, const Value& value, ArmConfig& config)
            {
                const auto* found = std::get_if<infra::JsonArray>(&value);
                if (found == nullptr)
                    return Fail(path, "must be a list of six joints");

                auto joints = *found;
                std::size_t count{ 0 };
                for (const auto& joint : joints)
                {
                    if (count == jointCount)
                        return Fail(path, "must list six joints");
                    if (!ReadJoint(Element(path, count), joint, config, count))
                        return false;
                    ++count;
                }

                if (joints.Error())
                    return Fail(path, "is not valid JSON");
                if (count != jointCount)
                    return Fail(path, "must list six joints");
                return UniqueNodes(path, config.nodes);
            }

            bool UniqueNodes(const Path& path, const JointNodeIds& nodes)
            {
                for (std::size_t joint = 1; joint != jointCount; ++joint)
                    for (std::size_t earlier = 0; earlier != joint; ++earlier)
                        if (nodes[joint] == nodes[earlier])
                        {
                            auto message = Element(path, earlier);
                            Path repeats{ "repeats the node of " };
                            Append(repeats, message);
                            return Fail(Child(Element(path, joint), "node"), repeats);
                        }

                return true;
            }

            struct Joint
            {
                bool node{ false };
                bool limits{ false };
                bool motion{ false };
            };

            bool ReadJoint(const Path& path, const Value& value, ArmConfig& config, std::size_t joint)
            {
                Joint seen;
                if (!ReadMembers(path, value, [this, &seen, &config, joint](const Path& item, infra::JsonString key, const Value& member)
                        {
                            return ReadJointItem(item, key, member, seen, config, joint);
                        }))
                    return false;

                if (!seen.node)
                    return Missing(path, "node");
                if (!seen.limits)
                    return Missing(path, "positionLimitsRad");
                if (!seen.motion)
                    return Missing(path, "motionLimits");
                return true;
            }

            bool ReadJointItem(const Path& path, infra::JsonString key, const Value& value, Joint& seen, ArmConfig& config, std::size_t joint)
            {
                if (key == "node")
                    return Mark(seen.node, ReadNode(path, value, config.nodes[joint]));
                if (key == "transmission")
                    return ReadTransmission(path, value, config.transmissions[joint]);
                if (key == "positionLimitsRad")
                    return Mark(seen.limits, ReadLimits(path, value, config.positionLimits[joint]));
                if (key == "motionLimits")
                    return Mark(seen.motion, ReadMotion(path, value, config.motionLimits[joint], config.jerkLimits[joint]));
                if (key == "trackingError")
                    return ReadTracking(path, value, config.tracking[joint]);
                if (key == "collisionTorqueNm")
                    return ReadCollision(path, value, config.collision[joint]);
                if (key == "speedLimitRadPerS")
                    return ReadSpeedTrip(path, value, config.speedTrips[joint]);
                if (key == "torqueLimitNm")
                    return ReadTorqueLimit(path, value, config.torqueLimits[joint]);
                if (key == "motorController")
                    return ReadController(path, value, config.controllers[joint]);
                if (key == "motor")
                    return ReadMotor(path, value, config.motors[joint]);
                return Unknown(path);
            }

            bool ReadMotor(const Path& path, const Value& value, std::optional<JointMotor>& result)
            {
                JointMotor motor{};
                std::array<bool, motorParameterCount> seen{};
                if (!ReadMembers(path, value, [this, &motor, &seen](const Path& item, infra::JsonString key, const Value& member)
                        {
                            return ReadMotorItem(item, key, member, motor, seen);
                        }))
                    return false;

                for (std::size_t parameter = 0; parameter != motorParameterCount; ++parameter)
                {
                    if (seen[parameter])
                        continue;
                    if (!motor.preset.has_value() && parameter < requiredMotorParameterCount)
                        return Missing(path, infra::BoundedConstString{ motorUnits[parameter].key.data(), motorUnits[parameter].key.size() });
                    motor.settings.values[parameter] = motorPresets[motor.preset.value_or(0)].settings.values[parameter];
                }

                result = motor;
                return true;
            }

            bool ReadMotorItem(const Path& path, infra::JsonString key, const Value& value, JointMotor& motor, std::array<bool, motorParameterCount>& seen)
            {
                if (key == "preset")
                    return ReadPreset(path, value, motor.preset);

                const auto unit = std::ranges::find_if(motorUnits, [&key](const MotorUnit& candidate)
                    {
                        return key == infra::BoundedConstString{ candidate.key.data(), candidate.key.size() };
                    });
                if (unit == motorUnits.end())
                    return Unknown(path);

                const auto parameter = static_cast<std::size_t>(std::distance(motorUnits.begin(), unit));
                return Mark(seen[parameter], ReadMotorValue(path, value, parameter, motor.settings.values[parameter]));
            }

            bool ReadPreset(const Path& path, const Value& value, std::optional<std::size_t>& preset)
            {
                if (const auto* text = std::get_if<infra::JsonString>(&value))
                    for (std::size_t index = 0; index != motorPresets.size(); ++index)
                        if (*text == infra::BoundedConstString{ motorPresets[index].key.data(), motorPresets[index].key.size() })
                        {
                            preset = index;
                            return true;
                        }

                return Fail(path, "must be teknic-m2310p-ln-04k or anaheim-bly172s-24v-4000");
            }

            bool ReadMotorValue(const Path& path, const Value& value, std::size_t parameter, float& result)
            {
                const auto number = Number(value);
                if (!number.has_value())
                    return Fail(path, "must be a number");

                result = static_cast<float>(*number / static_cast<double>(motorUnits[parameter].scale));
                if (static_cast<MotorParameter>(parameter) == MotorParameter::polePairs && static_cast<float>(static_cast<int32_t>(result)) != result)
                    return Fail(path, "must be a whole number");

                const auto descriptor = can::FindParameter(motorParameters[parameter]);
                if (descriptor.has_value() && (result < descriptor->minimum || result > descriptor->maximum))
                    return Fail(path, "is outside the e-foc contract's range");
                return true;
            }

            bool ReadNode(const Path& path, const Value& value, uint16_t& node)
            {
                int32_t id{ 0 };
                if (!ReadInteger(path, value, 1, 255, "must be a whole number from 1 to 255", id))
                    return false;
                node = static_cast<uint16_t>(id);
                return true;
            }

            bool ReadTransmission(const Path& path, const Value& value, JointTransmission& transmission)
            {
                return ReadMembers(path, value, [this, &transmission](const Path& item, infra::JsonString key, const Value& member)
                    {
                        if (key == "ratio")
                            return ReadPositive(item, member, transmission.ratio);
                        if (key == "direction")
                            return ReadDirection(item, member, transmission.direction);
                        if (key == "offsetRad")
                            return ReadRadians(item, member, transmission.offset);
                        return Unknown(item);
                    });
            }

            bool ReadDirection(const Path& path, const Value& value, JointDirection& direction)
            {
                std::size_t index{ 0 };
                if (!ReadChoice(path, value, directionNames, "must be positive or negative", index))
                    return false;
                direction = index == 0 ? JointDirection::positive : JointDirection::negative;
                return true;
            }

            bool ReadRadians(const Path& path, const Value& value, foc::Radians& result)
            {
                float radians{ 0.0f };
                if (!ReadReal(path, value, radians))
                    return false;
                result = foc::Radians{ radians };
                return true;
            }

            bool ReadLimits(const Path& path, const Value& value, JointLimit& limit)
            {
                std::array<bool, 2> seen{};
                if (!ReadMembers(path, value, [this, &seen, &limit](const Path& item, infra::JsonString key, const Value& member)
                        {
                            if (key == "lower")
                                return Mark(seen[0], ReadRadians(item, member, limit.lower));
                            if (key == "upper")
                                return Mark(seen[1], ReadRadians(item, member, limit.upper));
                            return Unknown(item);
                        }))
                    return false;

                if (!seen[0])
                    return Missing(path, "lower");
                if (!seen[1])
                    return Missing(path, "upper");
                if (!(limit.lower < limit.upper))
                    return Fail(Child(path, "lower"), "must be below upper");
                return true;
            }

            bool ReadMotion(const Path& path, const Value& value, JointMotionLimit& limit, std::optional<foc::RadiansPerSecondCubed>& jerk)
            {
                std::array<bool, 2> seen{};
                std::array<float, 2> values{};
                if (!ReadMembers(path, value, [this, &seen, &values, &jerk](const Path& item, infra::JsonString key, const Value& member)
                        {
                            if (key == "speedRadPerS")
                                return Mark(seen[0], ReadPositive(item, member, values[0]));
                            if (key == "accelerationRadPerS2")
                                return Mark(seen[1], ReadPositive(item, member, values[1]));
                            if (key == "jerkRadPerS3")
                                return ReadJerk(item, member, jerk);
                            return Unknown(item);
                        }))
                    return false;

                if (!seen[0])
                    return Missing(path, "speedRadPerS");
                if (!seen[1])
                    return Missing(path, "accelerationRadPerS2");
                limit = { foc::RadiansPerSecond{ values[0] }, foc::RadiansPerSecondSquared{ values[1] } };
                return true;
            }

            bool ReadJerk(const Path& path, const Value& value, std::optional<foc::RadiansPerSecondCubed>& jerk)
            {
                float number{ 0.0f };
                if (!ReadPositive(path, value, number))
                    return false;
                jerk = foc::RadiansPerSecondCubed{ number };
                return true;
            }

            bool ReadTracking(const Path& path, const Value& value, TrackingLimit& limit)
            {
                return ReadMembers(path, value, [this, &limit](const Path& item, infra::JsonString key, const Value& member)
                    {
                        float number{ 0.0f };
                        if (key == "windowRad")
                            return ReadPositive(item, member, number) && Store(limit.window, foc::Radians{ number });
                        if (key == "timeS")
                            return ReadTrackingTime(item, member, limit.time);
                        return Unknown(item);
                    });
            }

            bool ReadCollision(const Path& path, const Value& value, foc::NewtonMeter& limit)
            {
                float torque{ 0.0f };
                return ReadBounded(path, value, 1000.0f, "must be above 0 and at most 1000", torque) && Store(limit, foc::NewtonMeter{ torque });
            }

            bool ReadSpeedTrip(const Path& path, const Value& value, std::optional<foc::RadiansPerSecond>& limit)
            {
                float speed{ 0.0f };
                return ReadPositive(path, value, speed) && Store(limit, std::optional{ foc::RadiansPerSecond{ speed } });
            }

            bool ReadTorqueLimit(const Path& path, const Value& value, std::optional<foc::NewtonMeter>& limit)
            {
                float torque{ 0.0f };
                return ReadBounded(path, value, 1000.0f, "must be above 0 and at most 1000", torque) && Store(limit, std::optional{ foc::NewtonMeter{ torque } });
            }

            bool ReadTrackingTime(const Path& path, const Value& value, infra::Duration& time)
            {
                float seconds{ 0.0f };
                if (!ReadReal(path, value, seconds))
                    return false;
                if (!(seconds > 0.0f && seconds <= 10.0f))
                    return Fail(path, "must be above 0 and at most 10");
                time = std::chrono::round<infra::Duration>(std::chrono::duration<float>{ seconds });
                return true;
            }

            template<class T>
            static bool Store(T& target, const T& value)
            {
                target = value;
                return true;
            }

            bool ReadController(const Path& path, const Value& value, std::optional<ControllerSettings>& controller)
            {
                ControllerSettings settings{};
                std::array<bool, 3> seen{};
                if (!ReadMembers(path, value, [this, &seen, &settings](const Path& item, infra::JsonString key, const Value& member)
                        {
                            return ReadControllerItem(item, key, member, seen, settings);
                        }))
                    return false;

                if (!seen[0])
                    return Missing(path, "current");
                if (!seen[1])
                    return Missing(path, "speed");
                if (!seen[2])
                    return Missing(path, "position");
                controller = settings;
                return true;
            }

            bool ReadControllerItem(const Path& path, infra::JsonString key, const Value& value, std::array<bool, 3>& seen, ControllerSettings& settings)
            {
                std::size_t index{ 0 };
                if (key == "current")
                {
                    seen[0] = ReadChoice(path, value, currentNames, "must be pid, decoupledPid, deadbeat or slidingMode", index);
                    settings.current = static_cast<can::FocCurrentAlgorithm>(index);
                    return seen[0];
                }
                if (key == "speed")
                {
                    seen[1] = ReadChoice(path, value, speedNames, "must be pid, lqi, adrc or twoDof", index);
                    settings.speed = static_cast<can::FocSpeedAlgorithm>(index);
                    return seen[1];
                }
                if (key == "position")
                {
                    seen[2] = ReadChoice(path, value, positionNames, "must be pid, cascadeP, lqr, lqi or twoDof", index);
                    settings.position = static_cast<can::FocPositionAlgorithm>(index);
                    return seen[2];
                }
                if (key == "gains")
                    return ReadGains(path, value, settings);
                return Unknown(path);
            }

            bool ReadGains(const Path& path, const Value& value, ControllerSettings& settings)
            {
                return ReadMembers(path, value, [this, &settings](const Path& item, infra::JsonString key, const Value& member)
                    {
                        const auto name = std::ranges::find_if(gainNames, [&key](const char* gainName)
                            {
                                return key == gainName;
                            });
                        if (name == gainNames.end())
                            return Unknown(item);

                        const auto gain = static_cast<std::size_t>(std::distance(gainNames.begin(), name));
                        return ReadGain(item, member, gain, settings.gains[gain]);
                    });
            }

            bool ReadGain(const Path& path, const Value& value, std::size_t gain, float& result)
            {
                if (!ReadReal(path, value, result))
                    return false;

                const auto descriptor = can::FindParameter(gainParameters[gain]);
                if (descriptor.has_value() && (result < descriptor->minimum || result > descriptor->maximum))
                    return Fail(path, "is outside the e-foc contract's range");
                return true;
            }

            infra::BoundedString& problem;
        };

        class Writer
        {
        public:
            explicit Writer(infra::TextOutputStream& stream)
                : stream(stream)
            {}

            void Write(const ArmProfile& profile)
            {
                const auto& config = profile.config;
                stream << "{\n  \"format\": \"" << formatMarker << "\",\n  \"version\": " << profileVersion << ",\n  \"name\": ";
                Text(profile.name);
                stream << ",\n  \"controlPeriodMs\": " << static_cast<int64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(config.controlPeriod).count())
                       << ",\n  \"missedRequestsBeforeSilent\": " << static_cast<uint32_t>(config.missedRequestsBeforeSilent)
                       << ",\n  \"commandTimeoutPeriods\": " << static_cast<uint32_t>(config.commandTimeoutPeriods)
                       << ",\n  \"driveWatchdogPeriods\": " << static_cast<uint32_t>(config.driveWatchdogPeriods)
                       << ",\n  \"gravityPd\": ";
                Tuning(config.gravityPd);
                stream << ",\n  \"computedTorque\": ";
                Tuning(config.computedTorque);
                stream << ",\n  \"gravityPid\": ";
                Tuning(config.gravityPid);
                stream << ",\n  \"jointImpedance\": ";
                Impedance(config.jointImpedance);
                stream << ",\n  \"collisionObserverGainPerS\": ";
                Number(config.collisionObserverGain);
                stream << ",\n  \"motionProfile\": \"" << motionProfileNames[static_cast<std::size_t>(config.motionProfile)] << "\"";
                stream << ",\n  \"toolLimits\": {";
                auto limits = config.toolLimits;
                for (const auto& [name, field] : ToolLimitFields(limits))
                {
                    stream << " \"" << name << "\": ";
                    Number(*field);
                    stream << ",";
                }
                stream << " \"manipulabilityMin\": ";
                Number(config.toolLimits.manipulabilityMin);
                stream << " }";
                TeachPoints(config);
                stream << ",\n  \"joints\": [";

                for (std::size_t joint = 0; joint != jointCount; ++joint)
                {
                    stream << (joint == 0 ? "\n" : ",\n");
                    Joint(config, joint);
                }

                stream << "\n  ],\n  \"geometry\": {\n    \"opwMetres\": { ";
                Geometry(config.geometry);
                stream << "\n  },\n  \"massProperties\": {\n    \"links\": [";
                for (std::size_t link = 0; link != jointCount; ++link)
                {
                    stream << (link == 0 ? "\n      " : ",\n      ");
                    Mass(config.mass.links[link]);
                }
                stream << "\n    ],\n    \"payload\": ";
                Mass(config.mass.payload);
                stream << "\n  },\n  \"meshes\": {";
                const char* separator = " ";
                for (std::size_t index = 0; index != meshCount; ++index)
                    if (!profile.meshes[index].empty())
                    {
                        stream << separator << "\"" << meshNames[index] << "\": ";
                        Text(profile.meshes[index]);
                        separator = ", ";
                    }
                stream << " },\n  \"safety\": {";
                Safety(config.workspace);
                stream << "\n  }\n}\n";
            }

            void Safety(const WorkspaceConfig& workspace)
            {
                if (workspace.workspace.has_value())
                {
                    stream << "\n    \"workspaceM\": ";
                    Box(*workspace.workspace);
                    stream << ",";
                }
                stream << "\n    \"forbiddenZonesM\": [";
                for (std::size_t zone = 0; zone != workspace.forbiddenZoneCount; ++zone)
                {
                    stream << (zone == 0 ? " " : ", ");
                    Box(workspace.forbiddenZones[zone]);
                }
                stream << " ],\n    \"linkRadiiM\": [ ";
                for (std::size_t link = 0; link != workspaceCapsules; ++link)
                {
                    stream << (link == 0 ? "" : ", ");
                    Number(workspace.linkRadii[link]);
                }
                stream << " ],\n    \"selfCollisionMarginM\": ";
                Number(workspace.selfCollisionMargin);
                stream << ",\n    \"allowedLinkPairs\": [";
                for (std::size_t pair = 0; pair != workspace.allowedLinkPairCount; ++pair)
                    stream << (pair == 0 ? " [ " : ", [ ") << workspace.allowedLinkPairs[pair][0] + 1 << ", " << workspace.allowedLinkPairs[pair][1] + 1 << " ]";
                stream << " ]";
            }

            void Box(const WorkspaceBox& box)
            {
                static constexpr std::array<const char*, 3> axes{ "x", "y", "z" };
                stream << "{ \"min\": ";
                Named(axes, std::array<float, 3>{ box.min[0], box.min[1], box.min[2] });
                stream << ", \"max\": ";
                Named(axes, std::array<float, 3>{ box.max[0], box.max[1], box.max[2] });
                stream << " }";
            }

            void Mass(const MassProperties& properties)
            {
                stream << R"({ "massKg": )";
                Number(properties.mass);
                stream << R"(, "centreM": )";
                Named(centreNames, properties.centre);
                stream << R"(, "inertiaKgM2": )";
                Named(inertiaNames, properties.inertia);
                stream << " }";
            }

            template<std::size_t N>
            void Named(const std::array<const char*, N>& names, const std::array<float, N>& values)
            {
                for (std::size_t index = 0; index != N; ++index)
                {
                    stream << (index == 0 ? "{ \"" : ", \"") << names[index] << "\": ";
                    Number(values[index]);
                }
                stream << " }";
            }

            void TeachPoints(const ArmConfig& config)
            {
                stream << ",\n  \"teachPoints\": [";
                for (std::size_t point = 0; point != config.teachPointCount; ++point)
                {
                    stream << (point == 0 ? "\n    { \"name\": " : ",\n    { \"name\": ");
                    Text(config.teachPoints[point].name);
                    stream << ", \"jointsRad\": [ ";
                    for (std::size_t joint = 0; joint != jointCount; ++joint)
                    {
                        stream << (joint == 0 ? "" : ", ");
                        Number(config.teachPoints[point].joints[joint].Value());
                    }
                    stream << " ] }";
                }
                stream << (config.teachPointCount == 0 ? " ]" : "\n  ]");
            }

            void Tuning(const BandwidthTuning& tuning)
            {
                stream << "{ \"bandwidthRadPerS\": ";
                Number(tuning.bandwidth);
                stream << ", \"dampingRatio\": ";
                Number(tuning.dampingRatio);
                stream << " }";
            }

            void Impedance(const JointImpedanceTuning& tuning)
            {
                stream << "{ ";
                if (tuning.stiffness.has_value())
                {
                    stream << "\"stiffnessNmPerRad\": [";
                    for (std::size_t joint = 0; joint != jointCount; ++joint)
                    {
                        stream << (joint == 0 ? " " : ", ");
                        Number((*tuning.stiffness)[joint]);
                    }
                    stream << " ], ";
                }
                stream << "\"dampingRatio\": ";
                Number(tuning.dampingRatio);
                stream << " }";
            }

            void Geometry(const ArmGeometry& geometry)
            {
                const std::array<float, opwLengthNames.size()> lengths{ geometry.opw.a1, geometry.opw.a2, geometry.opw.b, geometry.opw.c1, geometry.opw.c2,
                    geometry.opw.c3, geometry.opw.c4 };
                for (std::size_t index = 0; index != lengths.size(); ++index)
                {
                    stream << (index == 0 ? "\"" : ", \"") << opwLengthNames[index] << "\": ";
                    Number(lengths[index]);
                }

                stream << " },\n    \"jointSigns\": [ ";
                for (std::size_t joint = 0; joint != jointCount; ++joint)
                    stream << (joint == 0 ? "" : ", ") << static_cast<int32_t>(geometry.opw.signs[joint]);
                stream << " ],\n    \"jointOffsetsRad\": [ ";
                for (std::size_t joint = 0; joint != jointCount; ++joint)
                {
                    stream << (joint == 0 ? "" : ", ");
                    Number(geometry.opw.offsets[joint]);
                }
                stream << " ],\n    \"tool\": ";
                Placement(geometry.tool);
                stream << ",\n    \"base\": ";
                Placement(geometry.base);
            }

            void Placement(const arm::Placement& placement)
            {
                const std::array<float, placementNames.size()> values{ placement.x, placement.y, placement.z, placement.roll, placement.pitch, placement.yaw };
                for (std::size_t index = 0; index != values.size(); ++index)
                {
                    stream << (index == 0 ? "{ \"" : ", \"") << placementNames[index] << "\": ";
                    Number(values[index]);
                }
                stream << " }";
            }

        private:
            void Joint(const ArmConfig& config, std::size_t joint)
            {
                const auto& transmission = config.transmissions[joint];
                stream << "    {\n      \"node\": " << static_cast<uint32_t>(config.nodes[joint]) << ",\n      \"transmission\": { \"ratio\": ";
                Number(transmission.ratio);
                stream << R"(, "direction": ")" << (transmission.direction == JointDirection::positive ? directionNames[0] : directionNames[1])
                       << R"(", "offsetRad": )";
                Number(transmission.offset.Value());
                stream << " },\n      \"positionLimitsRad\": { \"lower\": ";
                Number(config.positionLimits[joint].lower.Value());
                stream << ", \"upper\": ";
                Number(config.positionLimits[joint].upper.Value());
                stream << " },\n      \"motionLimits\": { \"speedRadPerS\": ";
                Number(config.motionLimits[joint].maxSpeed.Value());
                stream << ", \"accelerationRadPerS2\": ";
                Number(config.motionLimits[joint].maxAcceleration.Value());
                if (config.jerkLimits[joint].has_value())
                {
                    stream << ", \"jerkRadPerS3\": ";
                    Number(config.jerkLimits[joint]->Value());
                }
                stream << " },\n      \"trackingError\": { \"windowRad\": ";
                Number(config.tracking[joint].window.Value());
                stream << ", \"timeS\": ";
                Number(std::chrono::duration<float>{ config.tracking[joint].time }.count());
                stream << " },\n      \"collisionTorqueNm\": ";
                Number(config.collision[joint].Value());
                if (config.speedTrips[joint].has_value())
                {
                    stream << ",\n      \"speedLimitRadPerS\": ";
                    Number(config.speedTrips[joint]->Value());
                }
                if (config.torqueLimits[joint].has_value())
                {
                    stream << ",\n      \"torqueLimitNm\": ";
                    Number(config.torqueLimits[joint]->Value());
                }
                if (config.controllers[joint].has_value())
                    Controller(*config.controllers[joint]);
                if (config.motors[joint].has_value())
                    Motor(*config.motors[joint]);
                stream << "\n    }";
            }

            void Motor(const JointMotor& motor)
            {
                stream << ",\n      \"motor\": {";
                const char* separator = "\n";
                if (motor.preset.has_value())
                {
                    const auto key = motorPresets[*motor.preset].key;
                    stream << R"(
        "preset": ")" << infra::BoundedConstString{ key.data(), key.size() }
                           << R"(")";
                    separator = ",\n";
                }

                for (std::size_t parameter = 0; parameter != motorParameterCount; ++parameter)
                {
                    const auto key = motorUnits[parameter].key;
                    stream << separator << "        \"" << infra::BoundedConstString{ key.data(), key.size() } << "\": ";
                    Number(motor.settings.values[parameter], motorUnits[parameter].scale);
                    separator = ",\n";
                }

                stream << "\n      }";
            }

            void Controller(const ControllerSettings& settings)
            {
                stream << ",\n      \"motorController\": {\n        \"current\": \"" << currentNames[static_cast<std::size_t>(settings.current)]
                       << "\",\n        \"speed\": \"" << speedNames[static_cast<std::size_t>(settings.speed)] << "\",\n        \"position\": \""
                       << positionNames[static_cast<std::size_t>(settings.position)] << "\",\n        \"gains\": {";

                for (std::size_t gain = 0; gain != gainCount; ++gain)
                {
                    stream << (gain == 0 ? "\n" : ",\n") << "          \"" << gainNames[gain] << "\": ";
                    Number(settings.gains[gain]);
                }

                stream << "\n        }\n      }";
            }

            void Number(float value, float scale = 1.0f)
            {
                const auto magnitude = (value < 0.0f ? -static_cast<double>(value) : static_cast<double>(value)) * static_cast<double>(scale);
                const auto decimals = ShortestDecimals(value, magnitude, scale);
                const auto scaled = static_cast<uint64_t>(magnitude * powersOfTen[decimals] + 0.5);
                const auto unit = static_cast<uint64_t>(powersOfTen[decimals]);
                auto fraction = scaled % unit;

                if (value < 0.0f && scaled != 0)
                    stream << "-";
                stream << scaled / unit;

                if (fraction == 0)
                    return;

                auto digits = decimals;
                while (fraction % 10 == 0)
                {
                    fraction /= 10;
                    --digits;
                }

                stream << "." << infra::Width(digits, '0') << fraction;
            }

            static std::size_t ShortestDecimals(float value, double magnitude, float scale)
            {
                int32_t exponent{ 0 };
                auto mantissa = magnitude;
                while (mantissa >= 10.0)
                {
                    mantissa /= 10.0;
                    ++exponent;
                }
                while (mantissa > 0.0 && mantissa < 1.0)
                {
                    mantissa *= 10.0;
                    --exponent;
                }

                for (int32_t digits = 6; digits <= 9; ++digits)
                {
                    const auto decimals = static_cast<std::size_t>(std::clamp<int32_t>(digits - 1 - exponent, 0, 9));
                    const auto rounded = static_cast<double>(static_cast<uint64_t>(magnitude * powersOfTen[decimals] + 0.5)) / powersOfTen[decimals];
                    if (static_cast<float>((value < 0.0f ? -rounded : rounded) / static_cast<double>(scale)) == value)
                        return decimals;
                }

                return 9;
            }

            void Text(infra::BoundedConstString text)
            {
                stream << R"(")";
                for (const auto character : text)
                {
                    if (character == '"' || character == '\\')
                        stream << R"(\)";
                    stream << infra::BoundedConstString{ &character, 1 };
                }
                stream << R"(")";
            }

            infra::TextOutputStream& stream;
        };
    }

    std::optional<ArmProfile> ReadArmProfile(infra::BoundedConstString text, infra::BoundedString& problem)
    {
        return Reader{ problem }.Read(text);
    }

    void WriteArmProfile(const ArmProfile& profile, infra::TextOutputStream& stream)
    {
        Writer{ stream }.Write(profile);
    }
}
