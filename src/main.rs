use bitflags::bitflags;
use safe_drive::{
    context::Context, error::DynError, logger::Logger, pr_info,
};

bitflags! {
    pub struct SolenoidState: u16 {
        const FRONT = 0b0001;
        const MIDDLE = 0b0010;
        const REAR = 0b0100;
    }
}

fn main() -> Result<(), DynError>{
    let ctx = Context::new()?;
    let node = ctx.create_node("mecanum_air_riser", None, Default::default())?;
    let subscriber = node.create_subscriber::<drobo_interfaces::msg::SolenoidStateMsg>("solenoid_order", None)?;
    let logger = Logger::new("mecanum_air_riser");
    let mut selector = ctx.create_selector()?;
    selector.add_subscriber(
        subscriber, 
        Box::new(move |_msg| {
            
        }),
    );
        loop {
            selector.wait()?;
        }
}
