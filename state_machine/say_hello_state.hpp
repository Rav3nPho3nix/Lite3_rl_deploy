// Fichier say_hello_state.hpp
// Cree par Giuliano CAPITANO
// Permet d'effectuer l'action de "dire bonjour" selon le temps

#define DEBUG 0

#include "state_base.h"
#include <array>


// "Dire bonjour" est une action qui se réalise tant que l'on ne l'arrete pas

// 4 phases :
//      - Se mettre au sol
//      - Relever le haut du corps
//      - Deplacer la patte d'appui sous le corps
//      - Dire bonjour de l'autre patte avant
//      - Bloquer la sortie de 'dire bonjour' tant que l'on n'est pas en position
//      - Se relever

class SayHelloState : public StateBase {
    private:
        VecXf init_joint_pos_, init_joint_vel_, current_joint_pos_, current_joint_vel_;
        float time_stamp_record_, run_time_;
        VecXf goal_joint_pos_, kp_, kd_;
        MatXf joint_cmd_;

        // Etapes pour gerer le mouvements
        enum class StateMachineSteps {LAY, POS, ANIM, RISE, END};
        StateMachineSteps state = StateMachineSteps::LAY;
        
        // Temps pour se coucher
        float laying_down_duration;
        // Temps pour aller en position
        float positioning_duration;
        // Temps pour se lever
        float stand_duration;

        // Hauteurs lors du mouvement
        float height_target;
        float front_height_target;
        float hip_y_offset;

        // Parametres pour l'animation
        VecXf animation_holding_position;
        enum class AnimationPhases {LIFT, MOVE};
        AnimationPhases animation_phase = AnimationPhases::LIFT;
        float animation_time_stamp = 0.f;
        float lift_duration = 0.5f;

        void GetRobotJointValue(){
            current_joint_pos_ = ri_ptr_->GetJointPosition();
            current_joint_vel_ = ri_ptr_->GetJointVelocity();
            run_time_ = ri_ptr_->GetInterfaceTimeStamp();
        }

        void RecordJointData(){
            init_joint_pos_ = current_joint_pos_;
            init_joint_vel_ = current_joint_vel_;
            time_stamp_record_ = run_time_;
        }

        float GetCubicSplinePos(float x0, float v0, float xf, float vf, float t, float T){
            if(t >= T) return xf;
            float a, b, c, d;
            d = x0;
            c = v0;
            a = (vf*T - 2*xf + v0*T + 2*x0) / pow(T, 3);
            b = (3*xf - vf*T - 2*v0*T - 3*x0) / pow(T, 2);
            return a*pow(t, 3)+b*pow(t, 2)+c*t+d;
        }
        float GetCubicSplineVel(float x0, float v0, float xf, float vf, float t, float T){
            if(t >= T) return 0;
            float a, b, c;
            c = v0;
            a = (vf*T - 2*xf + v0*T + 2*x0) / pow(T, 3);
            b = (3*xf - vf*T - 2*v0*T - 3*x0) / pow(T, 2);
            return 3.*a*pow(t, 2) + 2.*b*t + c;
        }

        float GetHipYPosByHeight(float h){
            float l1 = cp_ptr_->thigh_len_;
            float l2 = cp_ptr_->shank_len_;
            float default_pos = (cp_ptr_->fl_joint_lower_(1)+cp_ptr_->fl_joint_upper_(1)) / 2.;
            if(fabs(h) >= l1 + l2) {
                std::cerr << "error height input" << std::endl;
                return 0;
            }
            float theta = -acos((l1*l1+h*h-l2*l2)/(2.*h*l1));
            theta = LimitNumber(theta, cp_ptr_->fl_joint_lower_(1), cp_ptr_->fl_joint_upper_(1));
            return theta;
        }

        float GetKneePosByHeight(float h){
            float l1 = cp_ptr_->thigh_len_;
            float l2 = cp_ptr_->shank_len_;
            float default_pos = (cp_ptr_->fl_joint_lower_(2)+cp_ptr_->fl_joint_upper_(2)) / 2.;
            if(fabs(h) >= l1 + l2) {
                std::cerr << "error height input" << std::endl;
                return 0;
            }
            float theta = M_PI-acos((l1*l1+l2*l2-h*h)/(2*l1*l2));
            theta = LimitNumber(theta, cp_ptr_->fl_joint_lower_(2), cp_ptr_->fl_joint_upper_(2));
            return theta;
        }

        // Couche le robot au sol
        void LayingDown(float t,  VecXf &planning_joint_pos, VecXf &planning_joint_vel) {
            float s = t / laying_down_duration;

            float h_max = 0.25f;
            float h_min = 0.10f;

            float h = h_min + (h_max - h_min) * (1.0f - s * s);
            
            // Gain d'amortissement
            float damping = 1.0f - std::min(t / laying_down_duration, 1.0f);

            // Position souhaitee
            VecXf target = VecXf::Zero(12);

            // indices pour parcours plus simple
            int indexes[4][3] = {
                {0, 1, 2},   // avant gauche
                {3, 4, 5},   // avant droit
                {6, 7, 8},   // arrière gauche
                {9, 10, 11}  // arrière droit
            };

            int side = 1;
            for (int k = 0; k < 4; k++) {
                
                int hip_x = indexes[k][0];
                int hip_y = indexes[k][1];
                int knee = indexes[k][2];

                float phase = damping;
                float h_eff = h * (0.6f + 0.4f * phase);

                target(hip_x) = side * height_target;
                target(hip_y) = GetHipYPosByHeight(h_eff);
                target(knee) = GetKneePosByHeight(h_eff);
                
                side *= -1;
            }

            for (int i = 0; i < 12; i++) {
                planning_joint_pos(i) = (1.0f - 0.1f) * current_joint_pos_(i) + 0.1f * target(i);
                planning_joint_vel(i) = 0.0f;
            }
        }

        // Met le robot en position
        void Positioning(float t, VecXf &planning_joint_pos, VecXf &planning_joint_vel) {
            float T = 1.0f;
            float s = t / T;
            s = std::min(std::max(s, 0.f), 1.f);
            s = s * s * (3.0f - 2.0f * s);

            VecXf target = VecXf::Zero(12);

            int indexes[4][3] = {
                {0,1,2},
                {3,4,5},
                {6,7,8},
                {9,10,11}
            };

            // Pattes avant
            for (int k = 0; k < 2; k++) {
                int hip_x = indexes[k][0];
                int hip_y = indexes[k][1];
                int knee  = indexes[k][2];

                float side = (k == 0) ? 1.f : -1.f;

                target(hip_x) = side * 0.4f;

                // Decalage pour prendre appui au sol plus proche du cente du corps
                // -0.8 OK avec offset patte arriere = -0.3
                // -0.9 OK avec offset patte arriere = -0.4
                // -0.8 OK avec -0.4 pour pattes arriere compactes
                target(hip_y) = GetHipYPosByHeight(front_height_target) - 0.9f;
                target(knee) = GetKneePosByHeight(front_height_target);
            }

            // Pattes arriere
            for (int k = 2; k < 4; k++) {
                int hip_x = indexes[k][0];
                int hip_y = indexes[k][1];
                int knee  = indexes[k][2];

                float side = (k == 2) ? 1.f : -1.f;

                target(hip_x) = 0.1f * side;
                // Ajouter un decalage car sinon hip y reste aligne au corps
                // -0.3 OK avec offset patte avant = -0.8
                // -0.4 OK avec offset patte avant = -0.9
                // -0.3 OK avec -0.9 pour pattes arriere compactes
                target(hip_y) = GetHipYPosByHeight(height_target) - 0.4f;
                target(knee)  = GetKneePosByHeight(height_target);
            }

            for (int i = 0; i < 12; i++) {
                planning_joint_pos(i) =
                    (1.0f - s) * current_joint_pos_(i) + s * target(i);
                planning_joint_vel(i) = 0.0f;
            }
        }

        // Fait l'animation
        void Animating(VecXf &planning_joint_pos, VecXf &planning_joint_vel) {
            float t = run_time_ - animation_time_stamp;

            switch (animation_phase) {

                case AnimationPhases::LIFT: {
                    float s = std::min(t / lift_duration, 1.f);
                    planning_joint_pos = animation_holding_position;
                    planning_joint_vel = VecXf::Zero(12);

                    // Monter le coude du bras droit
                    planning_joint_pos(5) = animation_holding_position(5) + s * 0.9f;
                    // Decaler le bras gauche pour plus de stabilite
                    planning_joint_pos(0) = animation_holding_position(0) + s * 0.5;

                    if (t >= lift_duration) {
                        animation_phase = AnimationPhases::MOVE;
                        animation_time_stamp = run_time_;
                        // Fige la pose courante
                        animation_holding_position = current_joint_pos_;
#if(DEBUG)
                        std::cout << "Knee lift finished" << std::endl;
                        std::cout << "Starting animation" << std::endl;
#endif
                    }
                } break;

                case AnimationPhases::MOVE: {
                    float t_hold = run_time_ - animation_time_stamp;
                    float freq = 1.0f;
                    // Amplitudes pour le mouvement
                    float hip_y_alpha = 0.3f;
                    float knee_alpha = 0.4f;

                    float s = sin(2.f * M_PI * freq * t_hold);

                    planning_joint_pos = animation_holding_position;
                    planning_joint_vel = VecXf::Zero(12);

                    // Met a jour
                    planning_joint_pos(4) = animation_holding_position(4) + hip_y_alpha * s;
                    planning_joint_pos(5) = animation_holding_position(5) + knee_alpha * s;
                } break;
            }
        }

        // Se releve
        void Rising(float t, VecXf &planning_joint_pos, VecXf &planning_joint_vel) {
            if(t <= stand_duration) {
                for(int i=0;i<current_joint_pos_.rows();++i) {
                    planning_joint_pos(i) = GetCubicSplinePos(init_joint_pos_(i), init_joint_vel_(i), goal_joint_pos_(i), 0, t, stand_duration);
                    planning_joint_vel(i) = GetCubicSplineVel(init_joint_pos_(i), init_joint_vel_(i), goal_joint_pos_(i), 0, t, stand_duration);
                }
            }
            else {
                float new_time = t - stand_duration;
                float dt = 0.001;
                float plan_height = GetCubicSplinePos(cp_ptr_->pre_height_, 0, cp_ptr_->stand_height_, 0, new_time, stand_duration);
                float plan_height_next = GetCubicSplinePos(cp_ptr_->pre_height_, 0, cp_ptr_->stand_height_, 0, new_time+dt, stand_duration);
                float hipy_pos = GetHipYPosByHeight(plan_height);
                float hipy_vel = (GetHipYPosByHeight(plan_height_next) - hipy_pos) / dt;
                float knee_pos = GetKneePosByHeight(plan_height);
                float knee_vel = (GetKneePosByHeight(plan_height_next) - knee_pos) / dt;
                planning_joint_pos = Vec3f(0, hipy_pos, knee_pos).replicate(4, 1);
                planning_joint_vel = Vec3f(0, hipy_vel, knee_vel).replicate(4, 1);
            }
        }

    public:

        // Constructeur
        SayHelloState(const RobotType& robot_type, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr) : StateBase(robot_type, state_name, data_ptr) {
            goal_joint_pos_ = Vec3f(0., GetHipYPosByHeight(cp_ptr_->pre_height_), GetKneePosByHeight(cp_ptr_->pre_height_)).replicate(4, 1);
            kp_ = VecXf(12);
            kd_ = VecXf(12);     
            kp_ = cp_ptr_->swing_leg_kp_.replicate(4, 1);
            kd_ = cp_ptr_->swing_leg_kd_.replicate(4, 1);
            joint_cmd_ = MatXf::Zero(12, 5);
            joint_cmd_.col(0) = kp_;
            joint_cmd_.col(2) = kd_;

            laying_down_duration = cp_ptr_->say_hello_laying_down_duration_;
            positioning_duration = cp_ptr_->say_hello_positioning_duration_;
            stand_duration = cp_ptr_->stand_duration_;

            height_target = cp_ptr_->say_hello_height_target_;
            front_height_target = cp_ptr_->say_hello_front_height_target_;
            
            hip_y_offset = cp_ptr_->say_hello_hip_y_offset_;
        }

        // Destructeur
        // Ne fait rien
        ~SayHelloState() {}

        // Lorsque l'etat commence
        virtual void OnEnter() {

#if(DEBUG)
            std::cout << "'SayHello' OnEnter" << std::endl;
#endif

            GetRobotJointValue();
            RecordJointData();

            state = StateMachineSteps::LAY;

            animation_phase = AnimationPhases::LIFT;
            animation_time_stamp = run_time_;

            StateBase::msfb_.UpdateCurrentState(RobotMotionState::SayHello);
            uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
        }

        // Lorsque l'etat se termine
        virtual void OnExit() {
#if(DEBUG)
            std::cout << "'SayHello' OnExit" << std::endl;
#endif
        }

        void Run() {
            GetRobotJointValue();

            float t = run_time_ - time_stamp_record_;

            VecXf q(12), dq(12);

            switch (state) {

                // Si on se couche
                case StateMachineSteps::LAY: {
                    // Toujours pendant l'etape'
                    if (t <= laying_down_duration) {
                        LayingDown(t, q, dq);
                    }
                    // Etape suivante
                    else {
                        state = StateMachineSteps::POS;
                        RecordJointData();
#if(DEBUG)
                        std::cout << "LayingDown finished" << std::endl;
#endif
                    }
                } break;

                // Si on se positionne
                case StateMachineSteps::POS: {
                    // Toujours pendant l'etape
                    if (t <= positioning_duration) {
                        Positioning(t, q, dq);
                    } 
                    // Etape suivante
                    else {
                        state = StateMachineSteps::ANIM;
                        animation_holding_position = current_joint_pos_;
                        animation_time_stamp = run_time_;  // ← indispensable
                        q  = current_joint_pos_;
                        dq = VecXf::Zero(12);
#if(DEBUG)
                        std::cout << "Positioning finished" << std::endl;
#endif
                    }                    
                } break;
                
                // Si on fait l'animation
                case StateMachineSteps::ANIM: {
                    Animating(q, dq);
                } break;

                // Si on remonte
                case StateMachineSteps::RISE: {
                    // Si on est en train de remonter
                    if (t <= 1.8f * stand_duration) {
                        Rising(t, q, dq);
                    }
                    else {
                        state = StateMachineSteps::END;
#if(DEBUG)
                        std::cout << "Rising finished" << std::endl;
#endif
                    }
                } break;

                default: {
                    q = current_joint_pos_;
                    dq = VecXf::Zero(12);
                } break;
            }

            joint_cmd_.col(1) = q;
            joint_cmd_.col(3) = dq;
            ri_ptr_->SetJointCommand(joint_cmd_);
        }

        // Verifie si le robot pert le controle
        // true = aller en Joint Damping / securite
        // false = execution normale / tout est ok
        virtual bool LoseControlJudge() {
            if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping)) return true;
            return PostureUnsafeCheck();
        }

        // Verifications d'une posture securise
        bool PostureUnsafeCheck(){
            Vec3f rpy = ri_ptr_->GetImuRpy();
            if(fabs(rpy(0)) > 30./180*M_PI || fabs(rpy(1)) > 45./180*M_PI){
                std::cout << "posture value: " << 180./M_PI*rpy.transpose() << std::endl;
                return true;
            }
            return false;
        }

        // Prochain etat
        virtual StateName GetNextStateName() {
            // Si on est a la fin
            if (state == StateMachineSteps::END) {
                return StateName::kRLControl;
            }

            // Si on souhaite sortir
            // On rappuie sur le meme bouton
            if (uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::ExitSayHello)) {
                // Si on est en position
                if (state != StateMachineSteps::LAY) {
                    // On amorce la remontee
                    if (state != StateMachineSteps::RISE) {
                        state = StateMachineSteps::RISE;
                        RecordJointData();
                    }
                }
            }
            
            // Laisser l'action
            return StateName::kSayHello;
        }
};