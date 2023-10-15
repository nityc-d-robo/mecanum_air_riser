use safe_drive::{
    context::Context, error::DynError, logger::Logger, pr_info, qos::Profile, topic::publisher::Publisher,
};
use drobo_interfaces::srv::{SolenoidStateSrv, SolenoidStateSrv_Response};

struct SolenoidState {
    front: bool,
    middle: bool,
    rear: bool,
}

fn main() -> Result<(), DynError>{
    let ctx = Context::new()?;
    let node = ctx.create_node("mecanum_air_riser", None, Default::default())?;
    let server = node.create_server::<SolenoidStateSrv>("solenoid_order", Some(Profile::default()))?;
    let publisher = node.create_publisher::<drobo_interfaces::msg::SdLibMsg>("sd_driver_topic", None)?;
    let mut selector = ctx.create_selector()?;
    let logger = Logger::new("mecanum_air_riser");

    let mut s_state = SolenoidState{
        front: false,
        middle: false,
        rear: false,
    };

    selector.add_server(
        server, 
        Box::new(move |msg, _header| {
            let mut response = SolenoidStateSrv_Response::new().unwrap();
            pr_info!(logger, "recv: {:?}", msg);

            let state_request = if msg.state >= 0 {
                msg.state == 1
            }else{
                match msg.axle_position {
                    0 => !s_state.front,
                    1 => !s_state.middle,
                    2 => !s_state.rear,
                    _ => panic!()
                }
            };
            response.result = match msg.axle_position {
                0 => if !state_request && !s_state.middle || !s_state.middle && !s_state.rear || !state_request && !s_state.rear {
                    publish_sd_state(&logger, &publisher, msg.axle_position, state_request);
                    s_state.front = state_request;
                    true
                }else {
                    pr_info!(logger, "却下");
                    false
                },
                1 => if !s_state.front && !state_request || !state_request && !s_state.rear || !s_state.front && !s_state.rear {
                    publish_sd_state(&logger, &publisher, msg.axle_position, state_request);
                    s_state.middle = state_request;
                    true
                }else {
                    pr_info!(logger, "却下");
                    false
                }
                2 => if !s_state.front && !s_state.middle || !s_state.middle && !state_request || !s_state.front && !state_request {
                    publish_sd_state(&logger, &publisher, msg.axle_position, state_request);
                    s_state.rear = state_request;
                    true
                }else {
                    pr_info!(logger, "却下");
                    false
                }
                _ => panic!()
            };
            response.state = [s_state.front, s_state.middle, s_state.rear];
            response
        }),
    );

    loop {
        selector.wait()?;
    }
}

fn publish_sd_state(logger: &Logger, publisher: &Publisher<drobo_interfaces::msg::SdLibMsg>, _axle_position: u8, _state: bool) {
    let address = _axle_position / 2;
    let port = _axle_position % 2;
    let power = !_state as u16 * 999;
    pr_info!(logger, "許可, address: {}, port: {}, power: {}", address, port, power);
    let mut msg = drobo_interfaces::msg::SdLibMsg::new().unwrap();
    msg.address = address;
    msg.port =  port;
    msg.power1 = power;
    publisher.send(&msg).unwrap();
}