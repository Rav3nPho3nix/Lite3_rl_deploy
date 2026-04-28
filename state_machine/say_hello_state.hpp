// Fichier say_hello_state.hpp
// Cree par Giuliano CAPITANO
// Permet d'effectuer l'action de "dire bonjour" selon le temps

#include "state_base.h"

class SayHelloState : public StateBase {
    private:


    public:

        // Constructeur
        SayHelloState(const RobotType& robot_ype, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr) : StateBase(robot_type, state_name, data_ptr) {}

        // Destructeur
        // Ne fait rien
        ~SayHelloState() {}

        // Lorsque l'etat commence
        virtual void OnEnter() {}

        // Lorsque l'etat se termine
        virtual void OnExit() {}

        // Lorsque l'etat s'execute
        virtual void Run() {}

        // Robot necessite de changer d'etat
        // true = doit changer d'etat
        // false = execution normale
        virtual bool LoseControlJudge() {return false}

        // Prochain etat (si necessaire)
        // Je mets ici le 'jointDamping' car c'est l'etat de mise en sécurité
        virtual StateName GetNextStateName() {return StateName::kJointDamping}
}