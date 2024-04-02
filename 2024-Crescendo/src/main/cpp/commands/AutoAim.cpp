

#include <subsystems/VisionSubsystem.h>
#include <commands/AutoAim.h>

AutoAim::AutoAim(Arm* arm, VisionSubsystem* vision, Superstructure* superstructure) 
    : m_arm(arm), m_vision(vision), m_superstructure(superstructure) {
  
  AddRequirements(arm);
  AddRequirements(vision);
  AddRequirements(superstructure);
}

void AutoAim::Initialize() {
   
    //m_arm->getRelativePosition()-(-0.03+m_arm->getRawPosition()));
    m_vision->setLedOn(3);
    m_superstructure->aim(m_vision->getDistance(), 0, 0);

}

void AutoAim::End(bool interrupted) {
    
}