// Fichier say_hello_state.hpp
// Cree par Giuliano CAPITANO
// Permet d'effectuer l'action de "dire bonjour" selon le temps

#include "state_base.h"
#include <array>

// "Dire bonjour" est une action qui se réalise tant que l'on ne l'arrete pas

// 3 phases :
//      - Se mettre au sol
//      - Relever le haut du corps
//      - Deplacer la patte d'appui sous le corps
//      - Dire bonjour de l'autre patte avant
//      - Bloquer la sortie de 'dire bonjour' tant que l'on n'est pas en position

class SayHelloState : public StateBase {
    private:
        VecXf init_joint_pos_, init_joint_vel_, current_joint_pos_, current_joint_vel_;
        float time_stamp_record_, run_time_;
        VecXf kp_, kd_;
        MatXf joint_cmd_;
        
        // Nombre de positions de l'animation
        static constexpr unsigned short NUMBER_POSITION_ANIMATION = 14;
        
        // Position du 'dire bonjour'
        VecXf ready_position_;
        // Positions pour l'animation
        // VecXf animation_positions_[NUMBER_POSITION_ANIMATION] = {0};
        std::array<VecXf, NUMBER_POSITION_ANIMATION> animation_positions_;

        // Positions cote droit
        VecXf right_low_arm, rigth_high_arm;

        // Positions cote gauche
        VecXf left_low_arm, left_high_arm;

        // Booleens qui gerent les passagent des etats
        bool is_in_position = false;
        
        // Temps pour aller en position
        float ready_position_duration_;

        // Temps pour faire chaque position de l'animation
        float animation_duration_;

        // Compteur pour l'animation
        unsigned short animation_index = 0;

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

    public:

        // Constructeur
        SayHelloState(const RobotType& robot_type, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr) : StateBase(robot_type, state_name, data_ptr) {
            VecXf goal(12);
            
            /** Position 'dire bonjour'
                 0.172  -0.623  0.613
                -0.172  -0.623  0.613
                 0      -1.9    2.429
                 0      -1.9    2.429
            */
            // goal <<  0.172, -0.623, 0.613,
            //         -0.172, -0.623, 0.613,
            //          0,     -1.9,   2.429,
            //          0,     -1.9,   2.429; 
            // goal <<  0.172, -0.623, 0.613,
            //         -0.172, -0.623, 0.613,
            //          0,     -1.42,   1.925,
            //          0,     -1.42,   1.925;
            // goal <<  0.530, -0.623, 0.613,
            //         -0.530, -0.623, 0.613,
            //          0,     -1.9,   2.8,
            //          0,     -1.9,   2.8;        
            goal <<  0, -1.5, 2.8,
                     0, -1.5, 2.8,
                     0, -1.33,   2.8,
                     0, -1.33,   2.8;
            ready_position_ = goal;
            
            /** Position animation droite bas
                 0.172  -0.623  0.613
                -0.172  -0.623  1.225 <-
                 0      -1.9    2.429
                 0      -1.9    2.429 
            */
            goal <<  0.172, -0.623, 0.613,
                    -0.172, -0.623, 1.225,
                     0,     -1.9,   2.429,
                     0,     -1.9,   2.429;
            // goal <<  0.172, -0.623, 0.613,
            //         -0.172, -0.623, 1.225,
            //          0,     -1.42,   1.925,
            //          0,     -1.42,   1.925; 
            right_low_arm = goal;

            /** Position animation droite haut
                 0.172  -0.623  0.613
                -0.172  -0.623  1.925 <-
                 0      -1.9    2.429
                 0      -1.9    2.429
            */
            goal <<  0.172, -0.623, 0.613,
                    -0.172, -0.623, 1.925,
                     0,     -1.9,   2.429,
                     0,     -1.9,   2.429;
            // goal <<  0.172, -0.623, 0.613,
            //         -0.172, -0.623, 1.925,
            //          0,     -1.42,   1.925,
            //          0,     -1.42,   1.925; 
            rigth_high_arm = goal;

            /** Position animation gauche bas
                 0.172  -0.623  1.225 <-
                -0.172  -0.623  0.613
                 0      -1.9    2.429
                 0      -1.9    2.429 
            */
            goal <<  0.172, -0.623, 1.225,
                    -0.172, -0.623, 0.613,
                     0,     -1.9,   2.429,
                     0,     -1.9,   2.429;
            // goal <<  0.172, -0.623, 1.225,
            //         -0.172, -0.623, 0.613,
            //          0,     -1.42,   1.925,
            //          0,     -1.42,   1.925; 
            left_low_arm = goal;

            /** Position animation gauche haut
                 0.172  -0.623  1.925 <-
                -0.172  -0.623  0.613
                 0      -1.9    2.429
                 0      -1.9    2.429
            */
            goal <<  0.172, -0.623, 1.925,
                    -0.172, -0.623, 0.613,
                     0,     -1.9,   2.429,
                     0,     -1.9,   2.429;
            // goal <<  0.172, -0.623, 1.925,
            //         -0.172, -0.623, 0.613,
            //          0,     -1.42,   1.925,
            //          0,     -1.42,   1.925; 
            left_high_arm = goal;

            // Tableau des positions
            // animation_positions_[0] = ready_position_;
            // animation_positions_[1] = right_low_arm;
            // animation_positions_[2] = rigth_high_arm;
            // animation_positions_[3] = right_low_arm;
            // animation_positions_[4] = rigth_high_arm;
            // animation_positions_[5] = right_low_arm;
            // animation_positions_[6] = rigth_high_arm;
            // animation_positions_[7] = ready_position_;
            // animation_positions_[8] = left_low_arm;
            // animation_positions_[9] = left_high_arm;
            // animation_positions_[10] = left_low_arm;
            // animation_positions_[11] = left_high_arm;
            // animation_positions_[12] = left_low_arm;
            // animation_positions_[13] = left_high_arm;

            animation_positions_[0] = ready_position_;
            animation_positions_[1] = ready_position_;
            animation_positions_[2] = ready_position_;
            animation_positions_[3] = ready_position_;
            animation_positions_[4] = ready_position_;
            animation_positions_[5] = ready_position_;
            animation_positions_[6] = ready_position_;
            animation_positions_[7] = ready_position_;
            animation_positions_[8] = ready_position_;
            animation_positions_[9] = ready_position_;
            animation_positions_[10] = ready_position_;
            animation_positions_[11] = ready_position_;
            animation_positions_[12] = ready_position_;
            animation_positions_[13] = ready_position_;

            kp_ = VecXf(12);
            kd_ = VecXf(12);     
            kp_ = cp_ptr_->swing_leg_kp_.replicate(4, 1);
            kd_ = cp_ptr_->swing_leg_kd_.replicate(4, 1);
            joint_cmd_ = MatXf::Zero(12, 5);
            joint_cmd_.col(0) = kp_;
            joint_cmd_.col(2) = kd_;
            ready_position_duration_ = cp_ptr_->say_hello_ready_position_duration_;

            animation_duration_ = cp_ptr_->say_hello_animation_frame_duration_;
        }

        // Destructeur
        // Ne fait rien
        ~SayHelloState() {}

        // Lorsque l'etat commence
        virtual void OnEnter() {
            // DEBUG
            std::cout << "'SayHello' OnEnter" << std::endl;
            GetRobotJointValue();
            RecordJointData();
            StateBase::msfb_.UpdateCurrentState(RobotMotionState::SayHello);
            uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
        }

        // Lorsque l'etat se termine
        virtual void OnExit() {
            // DEBUG
            std::cout << "'SayHello' OnExit" << std::endl;
        }

        void Run() {
            GetRobotJointValue();

            float t = run_time_ - time_stamp_record_;

            VecXf planning_joint_pos(12);
            VecXf planning_joint_vel(12);

            float T = ready_position_duration_;

            // gain amortissement global
            float damping = 1.0f - std::min(t / T, 1.0f);

            float hip_x_target = 0.2f;

            // Descente
            if (t <= T) {
                float s = t / T;

                float h_max = 0.25f;
                float h_min = 0.10f;

                float h = h_min + (h_max - h_min) * (1.0f - s * s); 

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

                    target(hip_x) = side * hip_x_target;
                    target(hip_y) = GetHipYPosByHeight(h_eff);
                    target(knee) = GetKneePosByHeight(h_eff);
                    
                    side *= -1;
                }

                for (int i = 0; i < 12; i++) {
                    planning_joint_pos(i) = (1.0f - 0.1f) * current_joint_pos_(i) + 0.1f * target(i);
                    planning_joint_vel(i) = 0.0f;
                }
            }

            // Couche
            else {

                VecXf target = VecXf::Zero(12);

                // Hauteur de la base par rapport au sol
                float h = 0.15f;

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
                    int knee  = indexes[k][2];

                    target(hip_x) = side * hip_x_target;
                    target(hip_y) = GetHipYPosByHeight(h);
                    target(knee)  = GetKneePosByHeight(h);

                    side *= -1;
                }

                for (int i = 0; i < 12; i++) {
                    planning_joint_pos(i) = 0.9f * current_joint_pos_(i) + 0.1f * target(i);
                }
                planning_joint_vel = VecXf::Zero(12);
            }

            joint_cmd_.col(1) = planning_joint_pos;
            joint_cmd_.col(3) = planning_joint_vel;
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
            // Si on souhaite sortir
            // On rappuie sur le meme bouton
            if (uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::ExitSayHello)) {
                // On revient en RL
                return StateName::kRLControl;
            }
            
            // Laisser l'action
            return StateName::kSayHello;
        }
};