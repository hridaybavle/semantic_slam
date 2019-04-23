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

    new_landmark_for_mapping_.clear();

    Q_.resize(6,6);
    Q_.setZero();

    Q_(0,0) =  0.9;
    Q_(1,1) =  0.9;
    Q_(2,2) =  0.9;
    Q_(3,3) =  0.9;
    Q_(4,4) =  0.9;
    Q_(5,5) =  0.9;
    //Q_(6,6) =  10.9;

    //normal distribution for init pose 0,0,0. TODO: pass init pose to from the semantic slam classs
    //and also add the std_deviation which is now 0.01
    std::normal_distribution<float> dist_x(0, 0);
    std::normal_distribution<float> dist_y(0, 0);
    std::normal_distribution<float> dist_z(0, 0);
    std::normal_distribution<float> dist_roll(0, 0);
    std::normal_distribution<float> dist_pitch(0, 0);
    std::normal_distribution<float> dist_yaw(0, 0);

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

        //compute the inv of the prev pose
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

void particle_filter::AllObjectMapAndUpdate(std::vector<particle_filter::all_object_info_struct_pf> complete_object_info,
                                            Eigen::VectorXf &final_pose)
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
                rotation_mat = particle_filter_tools_obj_.transformNormalsToWorld(all_particles_[i].pose);

                particle_filter::landmark new_landmark;
                this->landmarkPoseInWorld(new_landmark,
                                          complete_object_info[j].pose,
                                          all_particles_[i].pose,
                                          rotation_mat);

                this->landmarkNormalsInWorld(new_landmark,
                                             complete_object_info[j].normal_orientation,
                                             all_particles_[i].pose,
                                             rotation_mat);



                //create and call the measurement model here
                Eigen::VectorXf expected_z;
                expected_z.resize(6,6);
                Eigen::MatrixXf H;
                this->LandmarkMeasurementModel(all_particles_[i],
                                               new_landmark,
                                               expected_z,
                                               H);

                Eigen::MatrixXf Hi = H.inverse().eval();
                new_landmark.sigma.resize(6,6);
                new_landmark.sigma =  Hi * Q_ * Hi.transpose();
                new_landmark.type                   = complete_object_info[j].type;
                new_landmark.plane_type             = complete_object_info[j].plane_type;
                //new_landmark.mapped_planar_points   = complete_object_info[j].planar_points;

                //                this->projectPointsOnPlane(all_particles_[i],
                //                                           new_landmark,
                //                                           complete_object_info[j],
                //                                           rotation_mat);

                all_particles_[i].landmarks.push_back(new_landmark);

            }

        }

        first_object_ = false;
        return;
    }

    //****************** matching and mapping part**********************************************/
    new_landmark_for_mapping_.clear();

    std::cout << "Here " << std::endl;

    //making the data association step mulithreaded for making it faster
#ifdef use_threading
    boost::thread_group data_ass_group;
    for (int i =0; i < num_particles_; ++i)
    {
        data_ass_group.create_thread(boost::bind(&particle_filter::AllDataAssociation, this,
                                                 i,
                                                 std::ref(complete_object_info)));
    }
    data_ass_group.join_all();

#else
    for (int i =0; i < num_particles_; ++i)
    {
        this->particle_filter::AllDataAssociation(i,complete_object_info);
    }
#endif

    //std::cout << "Here 1" << std::endl;

    //map all the new landmarks for all particles with multithreading
    if(new_landmark_for_mapping_.size() > 0)
    {
        //std::cout << "new landmark data " << new_landmark_for_mapping_.size() << std::endl;

#ifdef use_threading
        boost::thread_group mapping_group;

        if(new_landmark_for_mapping_.size() <= num_particles_)
        {
            for(int i =0; i < new_landmark_for_mapping_.size(); ++i)
            {

                mapping_group.create_thread(boost::bind(&particle_filter::MapNewLandmarksForEachParticle, this,
                                                        i,
                                                        std::ref(complete_object_info)));
            }
            mapping_group.join_all();
        }
        else
        {
            for (int i =0; i < new_landmark_for_mapping_.size(); ++i)
            {
                this->particle_filter::MapNewLandmarksForEachParticle(i,complete_object_info);
            }
        }

#else
        for (int i =0; i < new_landmark_for_mapping_.size(); ++i)
        {
            this->particle_filter::MapNewLandmarksForEachParticle(i,complete_object_info);
        }
#endif

    }

    //std::cout << "Here 2" << std::endl;

    //resampling
    this->AllDataResample(final_pose);

    //std::cout << "Here 3" << std::endl;


    return;

}

void particle_filter::AllDataAssociation(int i, std::vector<all_object_info_struct_pf> complete_object_info)
{

    //#ifdef use_threading
    //    boost::thread_group obj_ass_group;
    //    for(int j = 0; j < complete_object_info.size(); ++j)
    //    {
    //        obj_ass_group.create_thread(boost::bind(&particle_filter::ObjectLevelDataAssociation, this,
    //                                                i,
    //                                                j,
    //                                                std::ref(complete_object_info)));
    //    }
    //    obj_ass_group.join_all();

    //#else
    for(int j = 0; j < complete_object_info.size(); ++j)
        this->particle_filter::ObjectLevelDataAssociation(i, j, complete_object_info);
    //#endif

}

void particle_filter::ObjectLevelDataAssociation(int i, int j,
                                                 std::vector<all_object_info_struct_pf> complete_object_info)
{

    float current_object_weight = 1;
    {
        bool found_nearest_neighbour = false;
        float maha_distance=0;
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

                    //converting the landmark pose in world frame
                    Eigen::Matrix3f rotation_mat;
                    rotation_mat = particle_filter_tools_obj_.transformNormalsToWorld(all_particles_[i].pose);

                    landmark detected_object;
                    this->landmarkPoseInWorld(detected_object,
                                              complete_object_info[j].pose,
                                              all_particles_[i].pose,
                                              rotation_mat);

                    this->landmarkNormalsInWorld(detected_object,
                                                 complete_object_info[j].normal_orientation,
                                                 all_particles_[i].pose,
                                                 rotation_mat);


                    Eigen::MatrixXf sig = all_particles_[i].landmarks[k].sigma;
                    Eigen::MatrixXf Q   = H * sig * H.transpose() + Q_;

                    //get the actual measurements
                    Eigen::VectorXf actual_measurements;
                    actual_measurements.resize(6,6);
                    actual_measurements(0) = detected_object.mu(0);
                    actual_measurements(1) = detected_object.mu(1);
                    actual_measurements(2) = detected_object.mu(2);
                    actual_measurements(3) = detected_object.normal_orientation(0);
                    actual_measurements(4) = detected_object.normal_orientation(1);
                    actual_measurements(5) = detected_object.normal_orientation(2);
                    //actual_measurements(6) = complete_object_info[j].normal_orientation(3);

                    //calculate the diff (innovations)
                    Eigen::VectorXf z_diff;
                    z_diff.resize(6,6);
                    z_diff = actual_measurements - expected_measurements;

                    maha_distance = z_diff.transpose() * Q.inverse() * z_diff;

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

        if(found_nearest_neighbour == false)
        {
            new_landmarks landmark;
            landmark.particle_id = i;
            landmark.object_id   = j;

            landmark_lock_.try_lock();
            new_landmark_for_mapping_.push_back(landmark);
            landmark_lock_.unlock();

        }
        else if(found_nearest_neighbour == true)
        {

            found_nearest_neighbour = false;

            if(maha_distance_min > MAHA_DIST_THRESHOLD)
            {


                new_landmarks landmark;
                landmark.particle_id = i;
                landmark.object_id   = j;

                landmark_lock_.try_lock();
                new_landmark_for_mapping_.push_back(landmark);
                landmark_lock_.unlock();
            }
            else
            {

                //***********************************************************//
                //kalman gain
                Eigen::MatrixXf K = min_sig * min_H.transpose() * min_Q.inverse();

                particle_lock_.try_lock();
                all_particles_[i].landmarks[neareast_landmarks_id].mu    = all_particles_[i].landmarks[neareast_landmarks_id].mu + K * min_z_diff;
                all_particles_[i].landmarks[neareast_landmarks_id].sigma = all_particles_[i].landmarks[neareast_landmarks_id].sigma - K * min_H * min_sig;
                particle_lock_.unlock();

                float current_weight = exp(-0.5*min_z_diff.transpose()*min_Q.inverse()*min_z_diff)/sqrt(2 * M_PI * min_Q.determinant());
                //std::cout << "current weight " << current_weight << std::endl;

                current_object_weight += current_weight;
            }
        }

    }

    particle_lock_.try_lock();
    all_particles_[i].weight *= current_object_weight;
    particle_lock_.unlock();


}


void particle_filter::AllDataResample(Eigen::VectorXf &final_pose)
{

    std::vector<float> weights;
    weights.resize(num_particles_);

    float sum=0;

    for(int i= 0; i < num_particles_; ++i)
    {
        weights[i] = all_particles_[i].weight;
        sum += weights[i];
    }

    //float n_effective=0;
    //normalizing the weights
    for (int i = 0; i < num_particles_; ++i)
    {
        //the if is in order to avoid nans
        if(weights[i] != 0 && sum != 0)
        {
            all_particles_[i].weight = all_particles_[i].weight/sum;
            weights[i]               = weights[i]/sum;
            //n_effective                += pow(weights[i],-2);
        }
    }


    //resample only is the sum is not zero and there are not alot of zeros in the weights
    //otherwise it will return
    if(sum == 0)
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

    //filling up the h measurement matrix
    h(0) = new_landmark.mu(0);
    h(1) = new_landmark.mu(1);
    h(2) = new_landmark.mu(2);
    h(3) = new_landmark.normal_orientation(0);
    h(4) = new_landmark.normal_orientation(1);
    h(5) = new_landmark.normal_orientation(2);

    //filling up the H measurement matrix
    H.setZero(6,6);
    H(0,0) = 1;  H(0,1) = 0;  H(0,2) = 0;
    H(1,0) = 0;  H(1,1) = 1;  H(1,2) = 0;
    H(2,0) = 0;  H(2,1) = 0;  H(2,2) = 1;
    H(3,3) = 1;  H(3,4) = 0;  H(3,5) = 0;
    H(4,3) = 0;  H(4,4) = 1;  H(4,5) = 0;
    H(5,3) = 0;  H(5,4) = 0;  H(5,5) = 1;

}

void particle_filter::MapNewLandmarksForEachParticle(int i,
                                                     std::vector<all_object_info_struct_pf> complete_object_info)
{

    //computing the new landmarks for each particle
    {
        int particle_id = new_landmark_for_mapping_[i].particle_id;
        int object_id   = new_landmark_for_mapping_[i].object_id;


        Eigen::Matrix3f transformation_mat;
        transformation_mat = particle_filter_tools_obj_.transformNormalsToWorld(all_particles_[particle_id].pose);

        landmark new_landmark;
        this->landmarkPoseInWorld(new_landmark,
                                  complete_object_info[object_id].pose,
                                  all_particles_[particle_id].pose,
                                  transformation_mat);


        this->landmarkNormalsInWorld(new_landmark,
                                     complete_object_info[object_id].normal_orientation,
                                     all_particles_[particle_id].pose,
                                     transformation_mat);


        //create and call the measurement model here
        Eigen::VectorXf expected_z;
        expected_z.resize(6,6);
        Eigen::MatrixXf H;
        this->LandmarkMeasurementModel(all_particles_[particle_id],
                                       new_landmark,
                                       expected_z,
                                       H);

        Eigen::MatrixXf Hi = H.inverse().eval();

        new_landmark.sigma =  Hi * Q_ * Hi.transpose();
        new_landmark.type                   = complete_object_info[object_id].type;
        new_landmark.plane_type             = complete_object_info[object_id].plane_type;
        //new_landmark.mapped_planar_points   = complete_object_info[object_id].planar_points;

        //        this->projectPointsOnPlane(all_particles_[particle_id],
        //                                   new_landmark,
        //                                   complete_object_info[object_id],
        //                                   transformation_mat);

        particle_lock_.try_lock();
        all_particles_[particle_id].landmarks.push_back(new_landmark);
        particle_lock_.unlock();
    }

}

inline void particle_filter::landmarkPoseInWorld(landmark &l,
                                                 Eigen::Vector3f object_pose,
                                                 Eigen::VectorXf particle_pose,
                                                 Eigen::Matrix3f rotation_mat)
{

    //converting the landmark pose in world frame
    Eigen::Vector3f landmark_pose_in_world_frame, landmark_pose_in_cam_frame, landmark_pose_in_robot_frame ;

    landmark_pose_in_cam_frame(0) = object_pose(0);
    landmark_pose_in_cam_frame(1) = object_pose(1);
    landmark_pose_in_cam_frame(2) = object_pose(2);

    landmark_pose_in_robot_frame  = rotation_mat * landmark_pose_in_cam_frame;

    landmark_pose_in_world_frame(0) = landmark_pose_in_robot_frame(0) + particle_pose(0);
    landmark_pose_in_world_frame(1) = landmark_pose_in_robot_frame(1) + particle_pose(1);
    landmark_pose_in_world_frame(2) = landmark_pose_in_robot_frame(2) + particle_pose(2);

    //this is for filling the landmarks for each particle
    l.mu = landmark_pose_in_world_frame;
}

inline void particle_filter::landmarkNormalsInWorld(landmark &l,
                                                    Eigen::Vector4f object_normals,
                                                    Eigen::VectorXf particle_pose,
                                                    Eigen::Matrix3f rotation_mat)
{
    //converting the landmark normals in world frame
    Eigen::Vector3f landmark_normals_in_world_frame, landmark_normals_in_cam_frame;

    landmark_normals_in_cam_frame(0) = object_normals(0);
    landmark_normals_in_cam_frame(1) = object_normals(1);
    landmark_normals_in_cam_frame(2) = object_normals(2);

    landmark_normals_in_world_frame = rotation_mat * landmark_normals_in_cam_frame;

    l.normal_orientation(0) = landmark_normals_in_world_frame(0);
    l.normal_orientation(1) = landmark_normals_in_world_frame(1);
    l.normal_orientation(2) = landmark_normals_in_world_frame(2);

    //this is to convert the d distance in world frame
    l.normal_orientation(3) =  object_normals(3) -
            (particle_pose(0) * l.normal_orientation(0) +
             particle_pose(1) * l.normal_orientation(1) +
             particle_pose(2) * l.normal_orientation(2));

}


void particle_filter::projectPointsOnPlane(particle p,
                                           landmark& l,
                                           all_object_info_struct_pf complete_object_info,
                                           Eigen::Matrix3f rotation_mat)
{
    float proj_x, proj_y, proj_z;


    proj_x = l.normal_orientation(3) * l.normal_orientation(0);
    proj_y = l.normal_orientation(3) * l.normal_orientation(1);
    proj_z = l.normal_orientation(3) * l.normal_orientation(2);


    for(size_t i =0; i < complete_object_info.planar_points.points.size(); ++i)
    {

        pcl::PointXYZRGB points;

        Eigen::Vector3f object_pose;
        object_pose(0) = complete_object_info.planar_points.points[i].x;
        object_pose(1) = complete_object_info.planar_points.points[i].y;
        object_pose(2) = complete_object_info.planar_points.points[i].z;

        this->landmarkPoseInWorld(l,
                                  object_pose,
                                  p.pose,
                                  rotation_mat);
        points.x   = l.mu(0);
        points.y   = l.mu(1);
        points.z   = l.mu(2);
        points.rgb = complete_object_info.planar_points.points[i].rgb;

        l.mapped_planar_points.push_back(points);
    }
}

