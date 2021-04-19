
#include <string>
#include <utility>
#include <vector>

#include "PololuDriver.hpp"

#include "engine/core/assert.hpp"
#include "engine/alice/message.hpp"
#include "engine/core/logger.hpp"
#include "engine/core/tensor/universal_tensor.hpp"
#include "engine/gems/tensor/utils.hpp"
#include "messages/tensor.hpp"
#include "packages/pololu/gems/maestro.hpp"

#include "packages/map/KinematicTree.hpp"


namespace isaac
{
    namespace pololu
    {
        namespace
        {

            constexpr int kNumChannels = 12;
        }
        void PololuDriver::start()
        {
            //TODO
            // Get the kinematic_tree object and validate against hardware
            device = Maestro(MaestroConfig(get_pololu_type(), get_protocol(), get_device(), get_device_number()));
            initStateSchema();
            tickPeriodically();
        }
        void PololuDriver::tick()
        {
            if (rx_command().available())
            {
                const int64_t time = rx_command().acqtime();
                if (!last_command_time_ || time > *last_command_time_)
                {
                    // If message is the first received message (last_command_time_ not initialized) or a more
                    // recent command, parse and send to arm.
                    last_command_time_ = time;

                    parseCommand();
                }
                else
                {
                    LOG_DEBUG("COMMAND TIME INCONSISTENCY");
                }
            }
            publishState();
        }
        void PololuDriver::stop()
        {
            device.disconnect();
        }

        void PololuDriver::initStateSchema()
        {
            std::vector<composite::Quantity> quantities;
            for (int i = 0; i < kNumChannels; i++)
            {
                quantities.push_back(
                    composite::Quantity::Scalar(channel_names_[i], composite::Measure::kPosition)); //Need 12 Channel names
                quantities.push_back(composite::Quantity::Scalar(channel_names_[i], composite::Measure::kSpeed));
            }
            state_schema_ = composite::Schema(std::move(quantities));
        }

        void PololuDriver::initCommandParser()
        {
            command_parser_.requestSchema(composite::Schema(channel_names_, composite::Measure::kPosition));
            return;
        }
        // Parses command from Composite message or Read command and write to pololu
        void PololuDriver::parseCommand()
        {
            LOG_DEBUG("parseCommand START");
            initCommandParser();

        VectorXd command(kNumChannels);
            if (!command_parser_.parse(rx_command().getProto(), rx_command().buffers(), command))
            {
                reportFailure("Fails to parse joint command");
                return;
            }
            commands_.clear();
            commands_.reserve(channel_names_.size());

            //This was needed as we had to convert our commands first and then send, use commands_ vector after conversion
            /*u_short goal_position = command(0);
            commands_.push_back(goal_position);*/
            for (size_t i; i < channel_names_.size(); i++)
            {
                LOG_DEBUG("command -> commands_ %i ", command(i));
                commands_.push_back((u_short)(command(i)));
            }
            /* Send command to Driver*/
            // bool success = true;
            for (size_t i = 0; i < channel_names_.size(); i++)
            {
                LOG_DEBUG("Maestro Channel id %i, Command given %i ", channel_names_[i], commands_[i]);
                device.setTargetOnChannel(i, commands_[i]); //driver call
            }
            //Call to Error Function
            //Error Handling

            for (int i = 0; i < 7; i++)
            {
                show("Maestro_servo" + std::to_string(i + 1) + ".command", command(i));
            }
        }

        void PololuDriver::publishState()
        {
            LOG_DEBUG("INSIDE PUBLISH STATE ");
            const int64_t acqtime = getTickTimestamp();
            int offset = 0;
            u_short state;
            Tensor1d state_data(kNumChannels);
            for (int i = 0; i < kNumChannels; i++)
            {
                device.getPositionOnChannel(i, state);
                state_data(offset++) = state;

                const std::string state = "state_motor_" + std::to_string(i + 1);
                show(state + ".state", state);
            }
            auto arm_proto_builder = tx_state().initProto();
            composite::WriteSchema(state_schema_, arm_proto_builder);
            ToProto(std::move(state_data), arm_proto_builder.initValues(), tx_state().buffers());
            tx_state().publish(acqtime);
        }

    }
}