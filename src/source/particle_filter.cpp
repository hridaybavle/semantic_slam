#include "particle_filter.h"

particle_filter::particle_filter()
{
    std::cout << "particle filter constructor " << std::endl;
}

particle_filter::~particle_filter()
{
    std::cout << "particle filter destructor " << std::endl;
}


inline bool comparator(const particle_filter::obs_vector& lhs, const particle_filter::obs_vector& rhs)
{
    return lhs.maha_dist < rhs.maha_dist;
}

inline bool highToLow(const float& lhs, const float& rhs)
{
    return lhs > rhs;
}

inline bool particlesHighToLow(const particle_filter::particle& lhs, particle_filter::particle& rhs)
{
    return lhs.weight > rhs.weight;
}

inline bool landmarkParticlesHighToLow(const particle_filter::landmark_particles& lhs, particle_filter::landmark_particles& rhs)
{
    return lhs.weight > rhs.weight;
}

inline bool comparatorlowToHigh(const float& lhs, const float& rhs)
{
    return lhs < rhs;
}


std::vector<Eigen::VectorXf> particle_filter::init(int state_size, int num_particles, std::vector<object_info_struct_pf> mapped_objects)
{
    this->state_size_ = state_size;
    this->num_particles_ = num_particles;

    std::default_random_engine gen;
    //std::default_delete gen;

    // ----------------------------------------------------------//

    std::vector<Eigen::VectorXf> init_pose_vec;

    init_pose_vec.resize(num_particles);

    particles_vec_.resize(num_particles);
    all_particles_.resize(num_particles);

    weights_.resize(num_particles);
    object_map_.clear();
    new_object_map_.clear();
    all_object_map_.clear();

    pose_increament_.setIdentity();

    first_object_ = true;

    first_horizontal_plane_ = false;
    first_vertical_plane    = false;

    first_chair_    = false;
    first_monitor_  = false;
    first_book_     = false;
    first_keyboard_ = false;
    first_chair_    = false;

    Q_.resize(6,6);
    Q_.setZero();

    Q_(0,0) =  5.9;
    Q_(1,1) =  5.9;
    Q_(2,2) =  5.9;
    Q_(3,3) =  10.9;
    Q_(4,4) =  10.9;
    Q_(5,5) =  10.9;
    //Q_(6,6) =  10.9;

    //normal distribution for init pose 0,0,0. TODO: pass init pose to from the semantic slam classs
    //and also add the std_deviation which is now 0.01
    std::normal_distribution<float> dist_x(0, 0);
    std::normal_distribution<float> dist_y(0, 0);
    std::normal_distribution<float> dist_z(0, 0);
    std::normal_distribution<float> dist_roll(0, 0);
    std::normal_distribution<float> dist_pitch(0, 0);
    std::normal_distribution<float> dist_yaw(0, 0);

    //    std::cout << "printing out the normal dist " << dist_x << std::endl
    //              << dist_y << std::endl
    //              << dist_z << std::endl
    //              << dist_roll << std::endl
    //              << dist_pitch << std::endl
    //              << dist_yaw << std::endl;

    for(int i =0; i < num_particles; ++i)
    {
        init_pose_vec[i].resize(state_size);

        vo_current_pose_.resize(state_size), vo_prev_pose_.resize(state_size);

        //std::cout << "state_vec_kk size " << state_vec_xkk_.size() << std::endl;

        //std::cout << "default random num " << gen << std::endl;

        vo_current_pose_.setZero(); vo_prev_pose_.setZero();


        all_particles_[i].pose.resize(state_size);
        all_particles_[i].pose(0) = dist_x(gen); //x
        all_particles_[i].pose(1) = dist_y(gen); //y
        all_particles_[i].pose(2) = dist_z(gen); //z
        all_particles_[i].pose(3) = dist_roll(gen); //roll
        all_particles_[i].pose(4) = dist_pitch(gen); //pitch
        all_particles_[i].pose(5) = dist_yaw(gen); //yaw
        all_particles_[i].weight  = 1;

        all_particles_[i].landmarks.clear();

        init_pose_vec[i](0) = all_particles_[i].pose(0);
        init_pose_vec[i](1) = all_particles_[i].pose(1);
        init_pose_vec[i](2) = all_particles_[i].pose(2);
        init_pose_vec[i](3) = all_particles_[i].pose(3);
        init_pose_vec[i](4) = all_particles_[i].pose(4);
        init_pose_vec[i](5) = all_particles_[i].pose(5);

    }

    return init_pose_vec;
}

void particle_filter::predictionVO(float deltaT,
                                   Eigen::VectorXf vo_pose_world,
                                   Eigen::VectorXf& final_pose)
{
    //random number generator
    std::default_random_engine gen;
    //std::default_delete gen;

    vo_current_pose_ = vo_pose_world;
    Eigen::VectorXf new_state_vec_xkk;
    new_state_vec_xkk.resize(6), new_state_vec_xkk.setZero();


    for(int i = 0; i < num_particles_; ++i)
    {
        Eigen::VectorXf prev_pose_inv, pose_increment;
        Eigen::VectorXf current_pose;
        prev_pose_inv.resize(6),  prev_pose_inv.setZero();
        pose_increment.resize(6), pose_increment.setZero();
        current_pose.resize(6), current_pose.setZero();
        pose_increament_.setIdentity();

        //----------------------this method is using the VO pose----------------------//

        //        //compute the inv of the prev pose
        prev_pose_inv = particle_filter_tools_obj_.computeVectorInv(vo_prev_pose_);
        //std::cout << "prev_pose_inv " << prev_pose_inv << std::endl;

        //compute the pose increment
        pose_increment = particle_filter_tools_obj_.computeDiffUsingHomoMethod(prev_pose_inv, vo_current_pose_);
        pose_increament_(0,3) =  pose_increment(0);
        pose_increament_(1,3) =  pose_increment(1);
        pose_increament_(2,3) =  pose_increment(2);

        //std::cout << "pose increment " << pose_increment << std::endl;

        //compute the pose using the prediction
        new_state_vec_xkk = particle_filter_tools_obj_.computeDiffUsingHomoMethod(all_particles_[i].pose, pose_increment);
        //std::cout << "new_state_vec_xkk " << " " << i << " " << new_state_vec_xkk << std::endl;


        //        std::normal_distribution<float> noise_x(0, 0.1);
        //        std::normal_distribution<float> noise_y(0, 0.1);
        //        std::normal_distribution<float> noise_z(0, 0.1);

        //adding gaussian noise to each normal distribution for each VO measurement
        std::normal_distribution<float> dist_x(new_state_vec_xkk(0), 0.0009);
        std::normal_distribution<float> dist_y(new_state_vec_xkk(1), 0.0009);
        std::normal_distribution<float> dist_z(new_state_vec_xkk(2), 0.0009);
        std::normal_distribution<float> dist_roll(new_state_vec_xkk(3), 0.0000);
        std::normal_distribution<float> dist_pitch(new_state_vec_xkk(4), 0.0000);
        std::normal_distribution<float> dist_yaw(new_state_vec_xkk(5), 0.0000);

        //updating each particle value based on the normal distribution
        all_particles_[i].pose(0) = dist_x(gen);
        all_particles_[i].pose(1) = dist_y(gen);
        all_particles_[i].pose(2) = dist_z(gen);
        all_particles_[i].pose(3) = dist_roll(gen);
        all_particles_[i].pose(4) = dist_pitch(gen);
        all_particles_[i].pose(5) = dist_yaw(gen);

        //--------------------------------------------------------------------------------------//

    }

    vo_prev_pose_ = vo_current_pose_;

    Eigen::VectorXf avg_pose;
    avg_pose.resize(state_size_), avg_pose.setZero();
    for(int i = 0; i < num_particles_; ++i)
    {
        avg_pose(0) += all_particles_[i].pose(0);
        avg_pose(1) += all_particles_[i].pose(1);
        avg_pose(2) += all_particles_[i].pose(2);
        avg_pose(3) += all_particles_[i].pose(3);
        avg_pose(4) += all_particles_[i].pose(4);
        avg_pose(5) += all_particles_[i].pose(5);

    }

    avg_pose(0) = avg_pose(0)/num_particles_;
    avg_pose(1) = avg_pose(1)/num_particles_;
    avg_pose(2) = avg_pose(2)/num_particles_;
    avg_pose(3) = avg_pose(3)/num_particles_;
    avg_pose(4) = avg_pose(4)/num_particles_;
    avg_pose(5) = avg_pose(5)/num_particles_;

    final_pose = avg_pose;

}

void particle_filter::AllObjectMapAndUpdate(std::vector<all_object_info_struct_pf> complete_object_info,
                                            Eigen::VectorXf &final_pose,
                                            Eigen::Matrix4f transformation_mat,
                                            std::vector<particle_filter::all_object_info_struct_pf>& mapped_objects)
{

    //check if its the first object and map it for all particles
    if(first_object_)
    {
        //computing the landmarks for each particle
        for(int i = 0; i < num_particles_; ++i)
        {
            //converting the pose of the objects to the world frame
            for(int j = 0; j < complete_object_info.size(); ++j)
            {
                Eigen::Matrix3f rotation_mat;
                rotation_mat.setZero();
                rotation_mat = particle_filter_tools_obj_.transformNormalsToWorld(all_particles_[i].pose);

                //converting the landmark pose in world frame
                Eigen::Vector3f landmark_pose_in_world_frame, landmark_pose_in_cam_frame, landmark_pose_in_robot_frame ;

                landmark_pose_in_cam_frame(0) = complete_object_info[j].pose(0);
                landmark_pose_in_cam_frame(1) = complete_object_info[j].pose(1);
                landmark_pose_in_cam_frame(2) = complete_object_info[j].pose(2);

                landmark_pose_in_robot_frame  = rotation_mat * landmark_pose_in_cam_frame;

                landmark_pose_in_world_frame(0) = landmark_pose_in_robot_frame(0) + all_particles_[i].pose(0);
                landmark_pose_in_world_frame(1) = landmark_pose_in_robot_frame(1) + all_particles_[i].pose(1);
                landmark_pose_in_world_frame(2) = landmark_pose_in_robot_frame(2) + all_particles_[i].pose(2);

                //converting the landmark normals in world frame
                Eigen::Vector3f landmark_normals_in_world_frame, landmark_normals_in_cam_frame;

                landmark_normals_in_cam_frame(0) = complete_object_info[j].normal_orientation(0);
                landmark_normals_in_cam_frame(1) = complete_object_info[j].normal_orientation(1);
                landmark_normals_in_cam_frame(2) = complete_object_info[j].normal_orientation(2);

                landmark_normals_in_world_frame = rotation_mat * landmark_normals_in_cam_frame;

                //this is for filling the landmarks for each particle
                landmark new_landmark;
                new_landmark.mu = landmark_pose_in_world_frame;

                new_landmark.normal_orientation(0) = landmark_normals_in_world_frame(0);
                new_landmark.normal_orientation(1) = landmark_normals_in_world_frame(1);
                new_landmark.normal_orientation(2) = landmark_normals_in_world_frame(2);

                //this is to convert the d distance in world frame
                new_landmark.normal_orientation(3) =   complete_object_info[j].normal_orientation(3) -
                        (all_particles_[i].pose(0) * new_landmark.normal_orientation(0) +
                         all_particles_[i].pose(1) * new_landmark.normal_orientation(1) +
                         all_particles_[i].pose(2) * new_landmark.normal_orientation(2));

                //create and call the measurement model here
                Eigen::VectorXf expected_z;
                expected_z.resize(6,6);
                Eigen::MatrixXf H;
                this->LandmarkMeasurementModel(all_particles_[i],
                                               new_landmark,
                                               expected_z,
                                               H);

                //                this->LandmarkNormalsMeasurementModel(all_particles_[i],
                //                                                      new_landmark,
                //                                                      expected_z,
                //                                                      H);


                Eigen::MatrixXf Hi = H.inverse().eval();

                //std::cout << "H matrix "  << H << std::endl;
                //std::cout << "H inverse " << Hi << std::endl;
                //std::cout << "O matrix "  << Q_ << std::endl;
                new_landmark.sigma.resize(6,6);
                new_landmark.sigma =  Hi * Q_ * Hi.transpose();

                //std::cout << "new landmark sigma " << new_landmark.sigma << std::endl;

                new_landmark.type               = complete_object_info[j].type;
                new_landmark.plane_type         = complete_object_info[j].plane_type;
                //new_landmark.normal_orientation = complete_object_info[j].normal_orientation;

                all_particles_[i].landmarks.push_back(new_landmark);
            }
        }

        first_object_ = false;
        return;

    }

    //****************** matching and mapping part**********************************************/

    std::vector<new_landmarks> new_landmark_for_mapping;
    new_landmark_for_mapping.clear();

    this->AllDataAssociation(complete_object_info, new_landmark_for_mapping);
    this->AllDataResample(complete_object_info, final_pose, mapped_objects, new_landmark_for_mapping);

    return;

}

void particle_filter::AllDataAssociation(std::vector<all_object_info_struct_pf> complete_object_info,
                                         std::vector<new_landmarks> &new_landmark_for_mapping)
{

    for (int i =0; i < num_particles_; ++i)
    {
        float current_object_weight = 0;
        for(int j = 0; j < complete_object_info.size(); ++j)
        {
            bool found_nearest_neighbour = false;
            float distance_normal = 0, maha_distance=0;
            float maha_distance_min = std::numeric_limits<float>::max();

            Eigen::VectorXf min_expected_measurements, min_z_diff;
            min_expected_measurements.resize(6,6), min_z_diff.resize(6,6);
            Eigen::MatrixXf min_H, min_sig, min_Q;


            //for(int k =0; k < all_object_map_.size(); ++k)
            int neareast_landmarks_id = -1;
            for(int k =0; k < all_particles_[i].landmarks.size(); ++k)
            {
                //single_matched_landmark = all_particles_[i].landmarks[k];

                if(complete_object_info[j].type == all_particles_[i].landmarks[k].type)
                {
                    if(complete_object_info[j].plane_type == all_particles_[i].landmarks[k].plane_type)
                    {

                        //                        distance_normal = particle_filter_tools_obj_.dist(complete_object_info[j].normal_orientation(0), all_particles_[i].landmarks[k].normal_orientation(0),
                        //                                                                          complete_object_info[j].normal_orientation(1), all_particles_[i].landmarks[k].normal_orientation(1),
                        //                                                                          complete_object_info[j].normal_orientation(2), all_particles_[i].landmarks[k].normal_orientation(2));


                        //if(distance_normal < 0.2)
                        {
                            //this step for matching with current landmarks
                            found_nearest_neighbour = true;

                            //calculating the mahalonobis distance
                            Eigen::VectorXf expected_measurements;
                            expected_measurements.resize(6,6);
                            Eigen::MatrixXf H;

                            //get the measurement model
                            this->LandmarkMeasurementModel(all_particles_[i],
                                                           all_particles_[i].landmarks[k],
                                                           expected_measurements,
                                                           H);

                            //                            this->LandmarkNormalsMeasurementModel(all_particles_[i],
                            //                                                                  all_particles_[i].landmarks[k],
                            //                                                                  expected_measurements,
                            //                                                                  H);


                            Eigen::MatrixXf sig = all_particles_[i].landmarks[k].sigma;
                            Eigen::MatrixXf Q   = H * sig * H.transpose() + Q_;


                            //get the actual measurements
                            Eigen::VectorXf actual_measurements;
                            actual_measurements.resize(6,6);
                            actual_measurements(0) = complete_object_info[j].pose(0);
                            actual_measurements(1) = complete_object_info[j].pose(1);
                            actual_measurements(2) = complete_object_info[j].pose(2);
                            actual_measurements(3) = complete_object_info[j].normal_orientation(0);
                            actual_measurements(4) = complete_object_info[j].normal_orientation(1);
                            actual_measurements(5) = complete_object_info[j].normal_orientation(2);
                            //actual_measurements(6) = complete_object_info[j].normal_orientation(3);

                            //                            Eigen::Vector4f actual_measurements;
                            //                            actual_measurements(0) = complete_object_info[j].normal_orientation(0);
                            //                            actual_measurements(1) = complete_object_info[j].normal_orientation(1);
                            //                            actual_measurements(2) = complete_object_info[j].normal_orientation(2);
                            //                            actual_measurements(3) = complete_object_info[j].normal_orientation(3);

                            //calculate the diff (innovations)
                            Eigen::VectorXf z_diff;
                            z_diff.resize(6,6);
                            z_diff = actual_measurements - expected_measurements;

                            maha_distance = z_diff.transpose() * Q.inverse() * z_diff;

                            //                            std::cout << "actual measurements "   << actual_measurements << std::endl;
                            //                            std::cout << "expected measurements " << expected_measurements << std::endl;
                            //                            std::cout << "maha distance "         << maha_distance << std::endl;
                            //                            std::cout << "end " << std::endl;

                            //insert the minimum Q, z_diff from above equations so you dont have to do the maths twice
                            if(maha_distance < maha_distance_min)
                            {
                                maha_distance_min       = maha_distance;
                                neareast_landmarks_id   = k;

                                //filling up the minimum variables for calculating the weights later
                                min_expected_measurements = expected_measurements;
                                min_H = H, min_Q = Q;
                                min_sig = sig;
                                min_z_diff = z_diff;

                            }

                        }

                    }

                }
            }

            if(found_nearest_neighbour == false)
            {

                new_landmarks landmark;
                landmark.particle_id = i;
                landmark.object_id   = j;

                new_landmark_for_mapping.push_back(landmark);

            }
            else if(found_nearest_neighbour == true)
            {

                found_nearest_neighbour = false;

                if(maha_distance_min > MAHA_DIST_THRESHOLD)
                {

                    new_landmarks landmark;
                    landmark.particle_id = i;
                    landmark.object_id   = j;
                    new_landmark_for_mapping.push_back(landmark);

                    continue;
                }
                else
                {

                    //***********************************************************//
                    //kalman gain
                    Eigen::MatrixXf K = min_sig * min_H.transpose() * min_Q.inverse();

                    all_particles_[i].landmarks[neareast_landmarks_id].mu    = all_particles_[i].landmarks[neareast_landmarks_id].mu + K * min_z_diff;
                    all_particles_[i].landmarks[neareast_landmarks_id].sigma = all_particles_[i].landmarks[neareast_landmarks_id].sigma - K * min_H * min_sig;


                    //std::cout << "Q  " << Q   << std::endl;
                    //std::cout << "Q_ " << Q_  << std::endl;

                    //std::cout << "landmark sigma " << all_particles_[i].landmarks[neareast_landmarks_id].sigma << std::endl;

                    float current_weight = exp(-0.5*min_z_diff.transpose()*min_Q.inverse()*min_z_diff)/sqrt(2 * M_PI * min_Q.determinant());
                    //std::cout << "current weight " << current_weight << std::endl;

                    current_object_weight += current_weight;
                }
            }

        }

        all_particles_[i].weight = current_object_weight;

    }


}


void particle_filter::AllDataResample(std::vector<all_object_info_struct_pf> complete_objec_info,
                                      Eigen::VectorXf &final_pose,
                                      std::vector<particle_filter::all_object_info_struct_pf>& mapped_objects,
                                      std::vector<new_landmarks> new_landmarks_for_mapping)
{

    std::vector<float> weights;
    weights.resize(num_particles_);

    float sum=0;
    float weight_counter=0;

    for(int i= 0; i < num_particles_; ++i)
    {
        weights[i] = all_particles_[i].weight;
        sum += weights[i];

        if(weights[i] == 0)
            weight_counter += 1;
    }


    //normalizing the weights
    for (int i = 0; i < num_particles_; ++i)
    {
        //the if is in order to avoid nans
        if(weights[i] != 0 && sum != 0)
        {
            all_particles_[i].weight = all_particles_[i].weight/sum;
            weights[i]               = weights[i]/sum;

        }
    }


    //std::cout << "sum  " << sum << std::endl;
    //map all the new landmarks for all particles first
    this->MapNewLandmarksForEachParticle(complete_objec_info, new_landmarks_for_mapping);


    //calculate the percentage of zeros in the data
    float percentage_of_zeros = 0;
    percentage_of_zeros =  weight_counter / num_particles_;

    //std::cout << "weight_counter" << weight_counter << std::endl;
    //std::cout << "percentage of zeros " << percentage_of_zeros << std::endl;

    //resample only is the sum is not zero and there are not alot of zeros in the weights
    //otherwise it will return
    if(sum == 0 || percentage_of_zeros  > 0.5)
    {
        std::cout << "not performing resampling " << std::endl;
        return;
    }


    std::vector<particle> resampled_particles;
    std::default_random_engine gen;

    //sorting the weight from high to low number
    std::vector<float> sorted_weights;
    sorted_weights = weights;

    std::sort(sorted_weights.begin(), sorted_weights.end(), highToLow);

    //sort all the particles based on their weights from high to low
    std::sort(all_particles_.begin(), all_particles_.end(), particlesHighToLow);

    std::discrete_distribution<> distr(sorted_weights.begin(), sorted_weights.end());
    //std::uniform_int_distribution<int> distr(sorted_weights.begin(), sorted_weights.end());

    resampled_particles.clear();
    resampled_particles.resize(num_particles_);

    //std::cout << "resampled state size " << resampled_state.size() << std::endl;
    for(int i = 0; i < num_particles_; ++i)
    {
        int number = distr(gen);
        resampled_particles[i] = all_particles_[number];
    }

    all_particles_ = resampled_particles;

    Eigen::VectorXf avg_pose;
    avg_pose.resize(state_size_), avg_pose.setZero();
    for(int i = 0; i < num_particles_; ++i)
    {
        avg_pose(0) += all_particles_[i].pose(0);
        avg_pose(1) += all_particles_[i].pose(1);
        avg_pose(2) += all_particles_[i].pose(2);
        avg_pose(3) += all_particles_[i].pose(3);
        avg_pose(4) += all_particles_[i].pose(4);
        avg_pose(5) += all_particles_[i].pose(5);

    }

    avg_pose(0) = avg_pose(0)/num_particles_;
    avg_pose(1) = avg_pose(1)/num_particles_;
    avg_pose(2) = avg_pose(2)/num_particles_;
    avg_pose(3) = avg_pose(3)/num_particles_;
    avg_pose(4) = avg_pose(4)/num_particles_;
    avg_pose(5) = avg_pose(5)/num_particles_;

    final_pose = avg_pose;

    return;
}

void particle_filter::LandmarkMeasurementModel(particle p,
                                               landmark new_landmark,
                                               Eigen::VectorXf &h,
                                               Eigen::MatrixXf &H)
{
    Eigen::Matrix3f rotation_mat;
    rotation_mat.setZero();
    rotation_mat = particle_filter_tools_obj_.transformNormalsToWorld(p.pose);

    Eigen::Vector3f landmark_pose_robot, landmark_pose_cam;

    landmark_pose_robot(0) = new_landmark.mu(0) - p.pose(0);
    landmark_pose_robot(1) = new_landmark.mu(1) - p.pose(1);
    landmark_pose_robot(2) = new_landmark.mu(2) - p.pose(2);

    landmark_pose_cam = rotation_mat.transpose().eval() * landmark_pose_robot;

    //filling up the h matrix with normal orientations first convert them to robot frame in which it is measured
    //converting the landmark normals in world frame
    Eigen::Vector3f landmark_normals_in_world_frame, landmark_normals_in_cam_frame;

    landmark_normals_in_world_frame(0) = new_landmark.normal_orientation(0);
    landmark_normals_in_world_frame(1) = new_landmark.normal_orientation(1);
    landmark_normals_in_world_frame(2) = new_landmark.normal_orientation(2);

    rotation_mat = particle_filter_tools_obj_.transformNormalsToWorld(p.pose);

    landmark_normals_in_cam_frame = rotation_mat.transpose().eval() * landmark_normals_in_world_frame;

    //    std::cout << "rotation_mat " << rotation_mat << std::endl;
    //    std::cout << "rotation_mat transpose " << rotation_mat.transpose().eval() << std::endl;
    //    std::cout << "landmark_normals_in_world_frame" << landmark_normals_in_world_frame << std::endl;
    //    std::cout << "landmark_normals_in_robot_frame" << landmark_normals_in_robot_frame << std::endl;
    //    std::cout << "endl " << std::endl;

    float distance_normal_in_cam = p.pose(0) * new_landmark.normal_orientation(0) +
            p.pose(1) * new_landmark.normal_orientation(1) +
            p.pose(2) * new_landmark.normal_orientation(2) +
            new_landmark.normal_orientation(3);


    //filling up the h measurement matrix
    h(0) = landmark_pose_cam(0);
    h(1) = landmark_pose_cam(1);
    h(2) = landmark_pose_cam(2);
    h(3) = landmark_normals_in_cam_frame(0);
    h(4) = landmark_normals_in_cam_frame(1);
    h(5) = landmark_normals_in_cam_frame(2);
    //h(6) = distance_normal_in_cam;

    //filling up the h measurement matrix
    rotation_mat = rotation_mat.transpose().eval();
    H.setZero(6,6);
    H(0,0) = rotation_mat(0,0); H(0,1) = rotation_mat(0,1); H(0,2) = rotation_mat(0,2);
    H(1,0) = rotation_mat(1,0); H(1,1) = rotation_mat(1,1); H(1,2) = rotation_mat(1,2);
    H(2,0) = rotation_mat(2,0); H(2,1) = rotation_mat(2,1); H(2,2) = rotation_mat(2,2);
    H(3,3) = rotation_mat(0,0); H(3,4) = rotation_mat(0,1); H(3,5) = rotation_mat(0,2);
    H(4,3) = rotation_mat(1,0); H(4,4) = rotation_mat(1,1); H(4,5) = rotation_mat(1,2);
    H(5,3) = rotation_mat(2,0); H(5,4) = rotation_mat(2,1); H(5,5) = rotation_mat(2,2);
    //H(6,3) = p.pose(0);         H(6,4) = p.pose(1);         H(6,5) = p.pose(2);
    //H(6,6) = 1;

}

void particle_filter::LandmarkNormalsMeasurementModel(particle p,
                                                      landmark new_landmark,
                                                      Eigen::Vector4f &h,
                                                      Eigen::MatrixXf &H)
{

    //filling up the h matrix with normal orientations first convert them to robot frame in which it is measured
    //converting the landmark normals in world frame
    Eigen::Vector3f landmark_normals_in_world_frame, landmark_normals_in_robot_frame;

    Eigen::Matrix3f rotation_mat;
    rotation_mat.setZero();
    rotation_mat = particle_filter_tools_obj_.transformNormalsToWorld(p.pose);

    landmark_normals_in_world_frame(0) = new_landmark.mu(0);
    landmark_normals_in_world_frame(1) = new_landmark.mu(1);
    landmark_normals_in_world_frame(2) = new_landmark.mu(2);

    rotation_mat = particle_filter_tools_obj_.transformNormalsToWorld(p.pose);

    landmark_normals_in_robot_frame = rotation_mat.transpose().eval() * landmark_normals_in_world_frame;

    //    std::cout << "rotation_mat " << rotation_mat << std::endl;
    //    std::cout << "rotation_mat transpose " << rotation_mat.transpose().eval() << std::endl;
    //    std::cout << "landmark_normals_in_world_frame" << landmark_normals_in_world_frame << std::endl;
    //    std::cout << "landmark_normals_in_robot_frame" << landmark_normals_in_robot_frame << std::endl;
    //    std::cout << "endl " << std::endl;

    float distance_normal_in_robot = p.pose(0) * new_landmark.mu(0) +
            p.pose(1) * new_landmark.mu(1) +
            p.pose(2) * new_landmark.mu(2) +
            new_landmark.mu(3);


    h(0) = landmark_normals_in_robot_frame(0);
    h(1) = landmark_normals_in_robot_frame(1);
    h(2) = landmark_normals_in_robot_frame(2);
    h(3) = distance_normal_in_robot;


    H.setZero(4,4);
    H(0,0) = rotation_mat(0,0); H(0,1) = rotation_mat(0,1); H(0,2) = rotation_mat(0,2);
    H(1,0) = rotation_mat(1,0); H(1,1) = rotation_mat(1,1); H(1,2) = rotation_mat(1,2);
    H(2,0) = rotation_mat(2,0); H(2,1) = rotation_mat(2,1); H(2,2) = rotation_mat(2,2);
    H(3,0) = p.pose(0);         H(3,1) = p.pose(1);         H(3,2) = p.pose(2);
    H(3,3) = 1;



}


void particle_filter::MapNewLandmarksForEachParticle(std::vector<all_object_info_struct_pf> complete_object_info,
                                                     std::vector<new_landmarks> &new_landmarks_for_mapping)
{

    //computing the new landmarks for each particle
    for(int i = 0; i < new_landmarks_for_mapping.size(); ++i)
    {
        int particle_id = new_landmarks_for_mapping[i].particle_id;
        int object_id   = new_landmarks_for_mapping[i].object_id;

        Eigen::Matrix3f transformation_mat;
        transformation_mat = particle_filter_tools_obj_.transformNormalsToWorld(all_particles_[particle_id].pose);

        //converting the landmark pose in world frame
        Eigen::Vector3f landmark_pose_in_world_frame, landmark_pose_in_cam_frame, landmark_pose_in_robot_frame;

        landmark_pose_in_cam_frame(0) = complete_object_info[object_id].pose(0);
        landmark_pose_in_cam_frame(1) = complete_object_info[object_id].pose(1);
        landmark_pose_in_cam_frame(2) = complete_object_info[object_id].pose(2);

        landmark_pose_in_robot_frame  = transformation_mat * landmark_pose_in_cam_frame;

        landmark_pose_in_world_frame(0) = landmark_pose_in_robot_frame(0) + all_particles_[particle_id].pose(0);
        landmark_pose_in_world_frame(1) = landmark_pose_in_robot_frame(1) + all_particles_[particle_id].pose(1);
        landmark_pose_in_world_frame(2) = landmark_pose_in_robot_frame(2) + all_particles_[particle_id].pose(2);

        //this is for filling the landmarks for each particle
        //converting the landmark normals in world frame
        Eigen::Vector3f landmark_normals_in_world_frame, landmark_normals_in_cam_frame;

        landmark_normals_in_cam_frame(0) = complete_object_info[object_id].normal_orientation(0);
        landmark_normals_in_cam_frame(1) = complete_object_info[object_id].normal_orientation(1);
        landmark_normals_in_cam_frame(2) = complete_object_info[object_id].normal_orientation(2);

        landmark_normals_in_world_frame = transformation_mat * landmark_normals_in_cam_frame;

        //this is for filling the landmarks for each particle
        landmark new_landmark;
        new_landmark.mu = landmark_pose_in_world_frame;

        new_landmark.normal_orientation(0) = landmark_normals_in_world_frame(0);
        new_landmark.normal_orientation(1) = landmark_normals_in_world_frame(1);
        new_landmark.normal_orientation(2) = landmark_normals_in_world_frame(2);

        //this is to convert the d distance in world frame
        new_landmark.normal_orientation(3) =   complete_object_info[object_id].normal_orientation(3) -
                (all_particles_[particle_id].pose(0) * new_landmark.normal_orientation(0) +
                 all_particles_[particle_id].pose(1) * new_landmark.normal_orientation(1) +
                 all_particles_[particle_id].pose(2) * new_landmark.normal_orientation(2));


        //create and call the measurement model here
        Eigen::VectorXf expected_z;
        expected_z.resize(6,6);
        Eigen::MatrixXf H;
        this->LandmarkMeasurementModel(all_particles_[particle_id],
                                       new_landmark,
                                       expected_z,
                                       H);

        //        this->LandmarkNormalsMeasurementModel(all_particles_[particle_id],
        //                                              new_landmark,
        //                                              expected_z,
        //                                              H);


        Eigen::MatrixXf Hi = H.inverse().eval();

        //std::cout << "H matrix "  << H << std::endl;
        //std::cout << "H inverse " << Hi << std::endl;
        //std::cout << "O matrix "  << Q_ << std::endl;
        new_landmark.sigma =  Hi * Q_ * Hi.transpose();

        //std::cout << "new landmark sigma " << new_landmark.sigma << std::endl;

        new_landmark.type               = complete_object_info[object_id].type;
        new_landmark.plane_type         = complete_object_info[object_id].plane_type;
        //new_landmark.normal_orientation = complete_object_info[j].normal_orientation;

        all_particles_[particle_id].landmarks.push_back(new_landmark);
    }

}
