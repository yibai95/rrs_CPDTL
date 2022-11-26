using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MaviGroundStation;
using System.IO;
using ProtoBuf;
using RRS.Tools.Network;
using System;
using UnityEngine.SceneManagement;
using System.Linq;
using RRS.Tools.Protobuf;
using Random=UnityEngine.Random;


public class CPDManager : MonoBehaviour
{
    public string skill_file_name = "2.txt";
    public string scenario_file_name = "2.txt";

    public GameObject user;
    public GameObject robot;
    public GameObject skillpoint_prefab;
    public GameObject cpdpoint_prefab;
    public GameObject nmpc_prefab;
    public GameObject plane_prefab;
    public GameObject nmpc_marker;
    public GameObject whiteboard;
    public GameObject benchmark_board;
    public GameObject point,point1,point2,point3,point4;
    


    private float originalWidth = 1000;
    private float originalHeight = 900;
    bool is_teleoperation_mode = false;
    public static bool is_network_inited = false;
    public int current_step = 0;
    public float delta_d = 0.6f;
    public Mode operation_mpde = Mode.SharedAutonomy;
    public Movo movo_ref;
    public Franka franka_ref;
    public string mode = "";
    public float timer = 0;


    List<RVector7> robot_point_list = new List<RVector7>();
    List<RVector7> skill_point_list = new List<RVector7>();
    List<RVector7> skill_point_list2 = new List<RVector7>();
    List<RVector7> nmpc_marker_list = new List<RVector7>();
    List<RVector7> re_traj = new List<RVector7>();
    List<RVector7> scenario_point_list = new List<RVector7>();
    List<RVector7> cpd_point_list = new List<RVector7>();
    List<RVector7> user_point_list = new List<RVector7>();
    RVector7 tele_robot_location = new RVector7();
    List<GameObject> skill_point_object_list = new List<GameObject>();
    List<GameObject> robot_point_object_list = new List<GameObject>();
    List<GameObject> cpd_point_object_list = new List<GameObject>();
    List<GameObject> user_point_object_list = new List<GameObject>();
    List<GameObject> nmpc_marker_object_list = new List<GameObject>();
    bool start_recording_skill = false;
    bool start_recording_scenario = false;
    bool update_render_cpd_result = false;
    bool is_switch_to_skill = false;
    Vector3 initial_user_point;
    Vector3 initial_nmpc_point;
    float next_scnario_time = 0;
    List<float> travel_times = new List<float>();
    float teleoperation_two_points_travel_time = 0;
    float average_teleoperation_two_points_travel_time = 0;
    RVector7 old_user_location = new RVector7();
    RVector7 old_robot_location = new RVector7();
    RVector7 last_robot_positon = new RVector7();
    RVector7 differ = new RVector7();
    RVector7 nmpc_marker_location = new RVector7();
    RVector7 initial_nmpc_marker = new RVector7();
    RVector7 nmpc_add = new RVector7();
    RVector7 eef_location = new RVector7();
    List<RVector7> franka_eef_list = new List<RVector7>();
    RVector7 eef_old_location = new RVector7();
    bool is_begin = false;
    float old_valid_time = 0;
    int reset_trail = 0;
    int scenario_index = 0;
    int nmpc_index = 0;
    int scenario_step_index = 0;
    int old_scenario_step_index = 0;
    bool is_play_scenario = false;
    bool is_nmpc = false;
    bool marker_initial = true;
    bool is_reproduce_franka = false;
    bool is_plane_register = false;
    bool is_plane_register2 = false;
    bool is_plane_register3 = false;
    public bool is_rigid = true;
    bool cpd_valid_result = false;
    int bench_index = 15;
    int disconnetion_index = 0;
    int bench_state = 0;
    bool is_go_to_skill = false;
    float p_max_distance = 0;
    float p_tracking_error = 0;
    float p_user_traj_len = 0;
    float p_robot_traj_len = 0;
    DateTime time_cpd_request_start = DateTime.Now;
    DateTime time_cpd_request_done = DateTime.Now;
    float total_max_distance = 0;
    float total_tracking_error = 0;
    float total_user_traj_len = 0;
    float total_robot_traj_len = 0;
    int max_step = 0;
    int max_step_non = 0;
    int iter = 0;
    //float d_eef = 0;
    bool record_vel = false;//franka step time
    float step_time = 0;
    float last_time = 0;
    List<float> franka_nativ_state = new List<float>();
    float cpdscale = 1;
    float old_cpdsacle = 0;//check if use 1 point or 2 points
    float link_limit = 0;//whiteboard limit
    float right_limit = 0;
    float up_limit = 0;
    float down_limit = 0;
    RVector7 start_point = new RVector7();
    bool XorZ;//point adjust
    bool is_plane_cpd = false;//cpd for plane limit
    bool is_adjust = false;//adjust the model
    bool large_or_small = false;
    List<RVector7> reproduce_list = new List<RVector7>();
    List<RVector7> new_skill = new List<RVector7>();
    //writing part
    bool is_writing = false;
    List<string> skill_file_list = new List<string>();
    int file_iter = 0;
    bool write_next = false;
    //cpd for synthesize skill
    List<RVector7> point_list = new List<RVector7>();
    bool is_skill_cpd = false;
    bool is_synthesize = false;
    bool finish_synthesize = false;
    bool finish_plane_register = false;
    List<RVector7> sythesize_skill = new List<RVector7>();
    List<int> count_list = new List<int>(); 
    List<Vector3> size_list = new List<Vector3>();
    int current_idx = 0;
    int start_idx = 0;
    int process_mode = 0;//select mode in fixedupdate: 1-NMPC, 2-Plane registeration, 3-Synthesize skill
    RVector7 boundary_mp = new RVector7();
    List<RVector7> convex_plane = new List<RVector7>();
    bool is_register_3D = false;//2D(false) or 3D register
    bool is_convex_plane = false;
    bool is_random_plane = false;
    RVector7 franka_mid_point = new RVector7();
    bool is_rotated = false;
    float angle = 0;
    List<RVector7> point_list2 = new List<RVector7>();
    bool back_to_initial_plane = false;
    int nonrigid_parameter = 5;
    List<RVector7> cpd_point_list_min = new List<RVector7>();//find the optimal norigid CPD result
    float average_min = 9999;
    int close_or_far = 0;
    int close_and_far = 0;
    int close_far = 99;
    int nmpc_execute_index = 0;
    List<string> joint_position_list = new List<string>();
    int eef_offset = 1;


    void resetScenario()
    {
        iter = 0;
        record_vel = false;
        step_time = 0;
        last_time = 0;
        is_rigid = true;
        //double uplimit[7] = {2.87, 1.74, 2.87, -0.06, 2.87, 3.6, 2.87};
        //double lowlimit[7] = {-2.87, -1.74, -2.87, -3.05, -2.87, -0.014, -2.87};
        //print(franka_ref.arm_+'1');
        
         cpdscale = 1;
         old_cpdsacle = 0;//check if use 1 point or 2 points
        link_limit = 0;//whiteboard limit
        right_limit = 0;
        up_limit = 0;
        down_limit = 0;
        start_point = new RVector7();
       
        is_plane_cpd = false;//cpd for plane limit
        is_adjust = false;//adjust the model
        large_or_small = false;
        reproduce_list = new List<RVector7>();
        new_skill = new List<RVector7>();
    //writing part
        is_writing = false;
        skill_file_list = new List<string>();
        file_iter = 0;
        write_next = false;
    //cpd for synthesize skill
        point_list = new List<RVector7>();
        is_skill_cpd = false;
        is_synthesize = false;
       finish_synthesize = false;
        finish_plane_register = false;
        sythesize_skill = new List<RVector7>();
       count_list = new List<int>(); 
        size_list = new List<Vector3>();
        current_idx = 0;
        start_idx = 0;
        process_mode = 0;//select mode in fixedupdate: 1-NMPC, 2-Plane registeration, 3-Synthesize skill
       boundary_mp = new RVector7();
        convex_plane = new List<RVector7>();
        is_register_3D = false;//2D(false) or 3D register
       is_convex_plane = false;
       is_random_plane = false;
       skill_point_list2 = new List<RVector7>();

        removePoints();

        scenario_step_index = 0;
        tele_robot_location = new RVector7();
        robot_point_list = new List<RVector7>();
        nmpc_marker_list = new List<RVector7>();
        re_traj = new List<RVector7>();
        skill_point_list = new List<RVector7>();
        scenario_point_list = new List<RVector7>();
        cpd_point_list = new List<RVector7>();
        user_point_list = new List<RVector7>();

        skill_point_object_list = new List<GameObject>();
        robot_point_object_list = new List<GameObject>();
        nmpc_marker_object_list = new List<GameObject>();
        cpd_point_object_list = new List<GameObject>();
        user_point_object_list = new List<GameObject>();

        is_begin = false;
        old_valid_time = 0;
        reset_trail = 0;
        scenario_index = 0;
        nmpc_index = 0;
        last_robot_positon = new RVector7();
        differ = new RVector7();
        //nmpc_marker_location = new RVector7();
        eef_location = new RVector7();
        eef_old_location = new RVector7();
        is_play_scenario = false;
        is_nmpc = false;
        is_reproduce_franka = false;
        cpd_valid_result = false;
        disconnetion_index = 0;

        foreach (var item in skill_point_object_list)
        {
            Destroy(item);
        }

        foreach (var item in cpd_point_object_list)
        {
            Destroy(item);
        }

        foreach (var item in nmpc_marker_object_list)
        {
            Destroy(item);
        }
        
        foreach (var item in user_point_object_list)
        {
            Destroy(item);
        }
        
        foreach (var item in robot_point_object_list)
        {
            Destroy(item);
        }

        old_user_location = new RVector7();
        old_robot_location = new RVector7();

        skill_point_object_list = new List<GameObject>();
        cpd_point_object_list = new List<GameObject>();
        is_switch_to_skill = false;
        is_teleoperation_mode = false;
        is_go_to_skill = false;

        user.transform.position = initial_user_point;
        nmpc_marker.transform.position = initial_nmpc_point;
        //user.transform.rotation = initial_user_point.rotation;

        cpd_point_list = new List<RVector7>();
        skill_point_list = new List<RVector7>();
        robot_point_list = new List<RVector7>();
        nmpc_marker_list = new List<RVector7>();
        user_point_list = new List<RVector7>();

        is_begin = false;
        old_valid_time = 0;
        current_step = 0;
        average_teleoperation_two_points_travel_time = 0;
        teleoperation_two_points_travel_time = 0;
        travel_times = new List<float>();

        old_user_location = new RVector7();
        old_robot_location = new RVector7();

        scenario_point_list = new List<RVector7>();

        is_play_scenario = false;
        is_nmpc = false;

        p_max_distance = 0;
        p_tracking_error = 0;
        p_user_traj_len = 0;
        p_robot_traj_len = 0;

        next_scnario_time = 0;

        scenario_index = 0;
        nmpc_index = 0;
        last_robot_positon = new RVector7();
        differ = new RVector7();
        nmpc_marker_location = new RVector7();

        reset_trail = 1;

        is_begin = false;
        is_teleoperation_mode = false;
        nonrigid_parameter = 5;

        point_list = new List<RVector7>();
        point_list2 = new List<RVector7>();
        RVector7 tp1 = new RVector7();
            tp1.x = point1.transform.position.x;
            tp1.y = point1.transform.position.y;
            tp1.z = point1.transform.position.z;
            RVector7 tp2 = new RVector7();
            tp2.x = point2.transform.position.x;
            tp2.y = point2.transform.position.y;
            tp2.z = point2.transform.position.z;
            RVector7 tp3 = new RVector7();
            tp3.x = point3.transform.position.x;
            tp3.y = point3.transform.position.y;
            tp3.z = point3.transform.position.z;
            RVector7 tp4 = new RVector7();
            tp4.x = point4.transform.position.x;
            tp4.y = point4.transform.position.y;
            tp4.z = point4.transform.position.z;
            point_list.Add(tp1);
            point_list.Add(tp2);
            point_list.Add(tp3);
            point_list.Add(tp4);
            point_list2.Add(tp1);
            point_list2.Add(tp2);
            point_list2.Add(tp3);
            point_list2.Add(tp4);

        removePoints();
        resetPoint();
       
    }

    public enum Mode
    {
        Nothing,
        SharedAutonomy,
        SharedAutonomyCPD,
    }

    void resetPoint()
    {
        tele_robot_location.x = user.transform.position.x;
        tele_robot_location.y = user.transform.position.y;
        tele_robot_location.z = user.transform.position.z;

        tele_robot_location.qx = user.transform.rotation.x;
        tele_robot_location.qy = user.transform.rotation.y;
        tele_robot_location.qz = user.transform.rotation.z;
        tele_robot_location.qw = user.transform.rotation.w;

        /*tele_robot_location.x = nmpc_marker.transform.position.x;
        tele_robot_location.y = nmpc_marker.transform.position.y;
        tele_robot_location.z = nmpc_marker.transform.position.z;

        tele_robot_location.qx = nmpc_marker.transform.rotation.x;
        tele_robot_location.qy = nmpc_marker.transform.rotation.y;
        tele_robot_location.qz = nmpc_marker.transform.rotation.z;
        tele_robot_location.qw = nmpc_marker.transform.rotation.w;*/

    }

    void convert2nmpc(List<RVector7> robot_list, RVector7 m){
        nmpc_marker_list = new List<RVector7>();
        RVector7 first = new RVector7();
        first.x = m.x;
        first.y = m.y;
        first.z = m.z;
        nmpc_marker_list.Add(first);
        for (int i = 1; i < robot_list.Count; i++){
            differ.x = robot_list[i].x - robot_list[i-1].x;
            differ.y = robot_list[i].y - robot_list[i-1].y;
            differ.z = robot_list[i].z - robot_list[i-1].z;

            
                nmpc_add = new RVector7();
                m.x += differ.x;
                m.y += differ.y;
                m.z += differ.z;
                nmpc_add.x = m.x;
                nmpc_add.y = m.y;
                nmpc_add.z = m.z;
                nmpc_marker_list.Add(nmpc_add);
            
            
        }
    }

    /*float find_angle(RVector7 p0, RVector7 p1, RVector7 p2){
        //p0 -> p1 -> p2
        float angle = 0; 
        Vector2 p01,p12 = new Vector2();
        p01.x = p1.x - p0.x;
        p01.y = p1.z - p0.z;
        //p01.z = p1.z - p0.z;
        p12.x = p2.x - p1.x;
        p12.y = p2.z - p1.z;
        //p12.z = p2.z - p1.z;
        angle = (p01.x*p12.x+p01.y*p12.y)/(Mathf.Sqrt(p01.x*p01.x+p01.y*p01.y)*Mathf.Sqrt(p12.x*p12.x+p12.y*p12.y));
        angle = Mathf.Acos(angle);
        return angle;

    }*/
    


    RVector7 find_start_error(List<RVector7> skill_list){
        RVector7 dist2left_down = new RVector7();//the distance between first point and left-down point
        float upx_point = 0;
        float downx_point = 0;
        float upy_point = 0;
        float downy_point = 0;
        

        int i = 0;
        foreach (var item in skill_list){
            if (i == 0){
                upx_point = item.x;
                downx_point = item.x;
                upy_point = item.y;
                downy_point = item.y;
            }
            i++;
            if (item.x > upx_point){
                upx_point = item.x;
            }
            if (item.x < downx_point){
                downx_point = item.x;
            }
            if (item.y > upy_point){
                upy_point = item.y;
            }
            if (item.y < downy_point){
                downy_point = item.y;
            }
        }
        dist2left_down.x = skill_list[0].x - downx_point;
        dist2left_down.y = skill_list[0].y - downy_point;
        dist2left_down.qx = upx_point - downx_point;//since we dont need the rotation
        dist2left_down.qy = upy_point - downy_point;

        return dist2left_down;
    }
    Vector2 calculate_points(List<RVector7> list){
        List<RVector7> statex = new List<RVector7>();
        List<RVector7> statey = new List<RVector7>();
        statex.Add(list[0]);
        statey.Add(list[0]);
        int idx = 0;
        int idy = 0;
        foreach (var item in list){
            if (item.x != list[idx].x) {
                statex.Add(item);
                idx++;}
            if (item.y != list[idy].y) {
                statey.Add(item);
                idy++;}
            
        }
        Vector2 num = new Vector2();
        num.x = statex.Count;
        num.y = statey.Count;
        return num;
    }

    List<RVector7> find_skill_frame(List<RVector7> skill_list,int num){
        List<RVector7> skill_frame = new List<RVector7>();
        float upx_point = 0;
        float downx_point = 0;
        float upy_point = 0;
        float downy_point = 0;
        float upz_point = 0;
        float downz_point = 0;

        int i = 0;
        foreach (var item in skill_list){
            if (i == 0){
                upx_point = item.x;
                downx_point = item.x;
                upy_point = item.y;
                downy_point = item.y;
                upz_point = item.z;
                downz_point = item.z;
                i++;
            }
            
            if (item.x > upx_point){
                upx_point = item.x;
            }
            if (item.x < downx_point){
                downx_point = item.x;
            }
            
                if (item.y > upy_point){
                    upy_point = item.y;
                }
                if (item.y < downy_point){
                downy_point = item.y;
                }
            
            
                if (item.z > upz_point){
                    upz_point = item.z;
                }
                if (item.z < downz_point){
                downz_point = item.z;
                }
            
        }
        float dx = Mathf.Abs(upx_point - downx_point)/num;
        float dy = Mathf.Abs(upy_point - downy_point)/num;
        float dz = Mathf.Abs(upz_point - downz_point)/num;
        RVector7 dl_point = new RVector7();
        dl_point.x = downx_point;
        dl_point.y = downy_point;
        dl_point.z = downz_point;
        skill_frame.Add(dl_point);
        for (int j = 0;;j++){
            RVector7 a = new RVector7();
            if (j < num){
                a.x = skill_frame[j].x;
                a.y = skill_frame[j].y;
                a.z = skill_frame[j].z + dz;
            }
            if (j >= num && j < num*2){
                a.x = skill_frame[j].x + dx;
                a.y = skill_frame[j].y;
                a.z = skill_frame[j].z;
            }
            if (j >= num*2 && j < num*3){
                a.x = skill_frame[j].x;
                a.y = skill_frame[j].y;
                a.z = skill_frame[j].z - dz;
            }
            if (j >= num*3 && j < num*4-1){
                a.x = skill_frame[j].x - dx;
                a.y = skill_frame[j].y;
                a.z = skill_frame[j].z;
            }
            if (j >= num*4-1) break;
            skill_frame.Add(a);
            
        }
        return skill_frame;
    }
    
    
    Vector3 find_lwh(List<RVector7> skill_list, int axis, int num){//0 for y axis, 1 for z axis
        Vector3 length_width_height = new Vector3();

        float upx_point = 0;
        float downx_point = 0;
        float upy_point = 0;
        float downy_point = 0;
        float upz_point = 0;
        float downz_point = 0;

        int i = 0;
        foreach (var item in skill_list){
            if (i == 0){
                upx_point = item.x;
                downx_point = item.x;
                upy_point = item.y;
                downy_point = item.y;
                upz_point = item.z;
                downz_point = item.z;
                i++;
            }
            
            if (item.x > upx_point){
                upx_point = item.x;
            }
            if (item.x < downx_point){
                downx_point = item.x;
            }
            if (axis == 0){
                if (item.y > upy_point){
                    upy_point = item.y;
                }
                if (item.y < downy_point){
                downy_point = item.y;
                }
            }
            if (axis == 1){
                if (item.z > upz_point){
                    upz_point = item.z;
                }
                if (item.z < downz_point){
                downz_point = item.z;
                }
            }
        }
        length_width_height.x = Mathf.Abs(upx_point - downx_point)/num;
        length_width_height.y = Mathf.Abs(upy_point - downy_point)/num;
        length_width_height.z = Mathf.Abs(upz_point - downz_point)/num;
        return length_width_height;
    }

    List<RVector7> add_points(List<RVector7> plist, int num){
        List<RVector7> points_list = new List<RVector7>();
        float dx14 = (plist[0].x - plist[3].x)/num;
        float dz14 = (plist[0].z - plist[3].z)/num;
        float dy14 = (plist[0].y - plist[3].y)/num;
        float dx23 = (plist[1].x - plist[2].x)/num;
        float dz23 = (plist[1].z - plist[2].z)/num;
        float dy23 = (plist[1].y - plist[2].y)/num;
        
        for (int i = 0;i<num;i++){
            RVector7 a = new RVector7(); 
            RVector7 b = new RVector7();
            a.x = plist[0].x - dx14*i;
            a.z = plist[0].z - dz14*i;
            a.y = plist[0].y - dy14*i;
            b.x = plist[1].x - dx23*i;
            b.z = plist[1].z - dz23*i;
            b.y = plist[1].y - dy23*i;
            float dx12 = (a.x - b.x)/num;
            float dz12 = (a.z - b.z)/num;
            float dy12 = (a.y - b.y)/num;
            //points_list.Add(a);
            for (int j = 0;j<num;j++){
                RVector7 aa = new RVector7();
                aa.x = a.x - dx12*j;
                aa.z = a.z - dz12*j;
                aa.y = a.y - dy12*j;
                points_list.Add(aa);
            }
            //points_list.Add(b);
            
        }

        return points_list;
    }

    Vector3 find_midpoint(List<RVector7> skill_list){
        Vector3 midpoint = new Vector3();
        float upx_point = 0;
        float downx_point = 0;
        float upy_point = 0;
        float downy_point = 0;
        float upz_point = 0;
        float downz_point = 0;

        int i = 0;
        foreach (var item in skill_list){
            if (i == 0){
                upx_point = item.x;
                downx_point = item.x;
                upy_point = item.y;
                downy_point = item.y;
                upz_point = item.z;
                downz_point = item.z;
                i++;
            }
            
            if (item.x > upx_point){
                upx_point = item.x;
            }
            if (item.x < downx_point){
                downx_point = item.x;
            }
            
            
                if (item.z > upz_point){
                    upz_point = item.z;
                }
                if (item.z < downz_point){
                downz_point = item.z;
                }
            if (item.y > upy_point){
                    upy_point = item.y;
                }
                if (item.y < downy_point){
                downy_point = item.y;
                }
            
        }
        midpoint.x = (upx_point + downx_point)/2;
        midpoint.y = (upy_point + downy_point)/2;
        midpoint.z = (upz_point + downz_point)/2;
        return midpoint; 

    }

    RVector7 find_frame_midpoint(List<RVector7> plist,int m){
        RVector7 frame_midpoint = new RVector7();
        frame_midpoint.y = plist[0].y;
        if (m == 1){
            frame_midpoint.x = (plist[0].x + plist[1].x)/2;
            frame_midpoint.z = (plist[0].z + plist[1].z)/2;
        }
        else if (m == 2){
            frame_midpoint.x = (plist[1].x + plist[2].x)/2;
            frame_midpoint.z = (plist[1].z + plist[2].z)/2;
        }
        else if (m == 3){
            frame_midpoint.x = (plist[2].x + plist[3].x)/2;
            frame_midpoint.z = (plist[2].z + plist[3].z)/2;
        }
        else if (m == 4){
            frame_midpoint.x = (plist[3].x + plist[0].x)/2;
            frame_midpoint.z = (plist[3].z + plist[0].z)/2;
        }
        return frame_midpoint;
    }

    

    List<RVector7> reproduce_traj(List<RVector7> traj, float sca){
        List<RVector7> re_traj = new List<RVector7>();
        
        //float p = (float)(traj.Count - 5) / (float)traj.Count;
        //print(p);
        int i = 0;
        while (true){
            RVector7 r = new RVector7();
            
            if (i == 0){
                
                r.x = traj[0].x;
                r.y = traj[0].y;
                r.z = traj[0].z;
                re_traj.Add(r);
                i = 1;
                }
            else{
                
                r.x = re_traj[i-1].x + (traj[i].x - traj[i-1].x)*sca;
                r.y = re_traj[i-1].y + (traj[i].y - traj[i-1].y)*sca;
                r.z = re_traj[i-1].z + (traj[i].z - traj[i-1].z)*sca;
                re_traj.Add(r);
                //print(r.z);
                i++;
            }
            if (i == traj.Count){
                break;
            }
            
        }
        return re_traj;
    }

    

    float find_point(Vector3 p1,Vector3 p2,Vector3 p3,Vector3 p4,RVector7 p, bool xz){
        //according to the position, return different int z = a*x +b
        float a12 = (p1.z - p2.z)/(p1.x - p2.x);
        float b12 = p1.z - a12 * p1.x;
        float a23 = (p2.z - p3.z)/(p2.x - p3.x);
        float b23 = p2.z - a23 * p2.x; 
        float a34 = (p3.z - p4.z)/(p3.x - p4.x);
        float b34 = p3.z - a34 * p3.x;
        float a41 = (p4.z - p1.z)/(p4.x - p1.x);
        float b41 = p4.z - a41 * p4.x;
        if (p.x*a12+b12<p.z){
            xz = false;
            return (p.z - b12)/a12+0.0001f;}
        else if (p.x*a23+b23<p.z){
            xz = true;
            return p.x*a23+b23+0.0001f;}
        else if (p.x*a34+b34<p.z){
            xz = false;
            return (p.z-b34)/a34+0.0001f;}
        
        else if (p.x*a41+b41>p.z){
            xz = true;
            return p.x*a41+b41+0.0001f;}
        //else if (p.x*a34+b34<p.z && p.x*a41+b41<p.z)return 6;
        //else if (p.x*a41+b41<p.z && p.x*a34+b34>p.z && p.x*a12+b12<p.z)return 7;
        //else if (p.x*a12+b12>p.z && p.x*a41+b41<p.z)return 8;
        else return 0;
        
    }

    Vector3 find_offset(List<RVector7> list1, List<RVector7> list2){
        Vector3 offset = new Vector3();
        Vector3 mp1 = find_midpoint(list1);
        Vector3 mp2 = find_midpoint(list2);
        print(mp1.y);
        print(mp2.y);
        
        Vector3 size1 = find_lwh(list1,0,1);
        Vector3 size2 = find_lwh(list2,0,1);
        if (mp1.x < mp2.x) offset.x = (size1.x + size2.x)*0.5f - Mathf.Abs(mp2.x - mp1.x);        
        else offset.x = (size1.x + size2.x)*0.5f + Mathf.Abs(mp2.x - mp1.x);
        
        offset.y = mp1.y - mp2.y;


        return offset;
    }

    void sythesize_new_skill(List<String> slist){
        int num = slist.Count;
        
        List<RVector7> temporal = new List<RVector7>();
        
        bool large_or_small = false;
        for (int i = 0; i < num;i++){
            FileStream fs = new FileStream(Application.dataPath + "/Benchmarks/skills/" + slist[i], FileMode.Open, FileAccess.Read);
            StreamReader sr = new StreamReader(fs);

            string a = sr.ReadToEnd();
            char[] ccc = new char[1];
            ccc[0] = '\n';

            char[] ccc2 = new char[1];
            ccc2[0] = ',';

            string[] list = a.Split(ccc);

            try
            {
                foreach (var item in list)
                {
                    if (item == "") continue;

                    string[] items = item.Replace("\r", string.Empty).Split(ccc2);

                    RVector7 point = new RVector7();
                    point.x = float.Parse(items[0]);
                    point.y = float.Parse(items[1]);
                    point.z = float.Parse(items[2]);

                    point.qx = float.Parse(items[3]);
                    point.qy = float.Parse(items[4]);
                    point.qz = float.Parse(items[5]);
                    point.qw = float.Parse(items[6]);

                    sythesize_skill.Add(point);
                    temporal.Add(point);
                    
                }
               
            }
            catch (Exception ee)
            {
                string me = ee.Message;
            }
            Vector3 size = new Vector3();
            size = find_lwh(temporal,0,1);
            size_list.Add(size); 
            count_list.Add(temporal.Count);
            /*if (i == 0) {              
                first_size = find_lwh(temporal,0,1);
            }
            else {
                current_size = find_lwh(temporal,0,1);
                //
                skill_point_list = temporal;
                if (current_size.x/first_size.x > 1 || current_size.y/first_size.y > 1)large_or_small = true;//skill larger
                bool end = true;
                while (end){
                if (large_or_small){
                    if (current_size.x/first_size.x > 1.5f || current_size.y/first_size.y > 1.5f){
                        print("Larger than the first letter");
                        
                        re_traj = reproduce_traj(temporal,0.9f);
                        is_skill_cpd = true;
                        doSkillCPD();
                    }
                    else{
                    print("Start adjust");
                    re_traj = reproduce_traj(temporal,0.99f);
                    is_skill_cpd = true;
                    
                    if (current_size.x/first_size.x <= 1 && current_size.y/first_size.y <= 1){
                        print("finish");
                        end = false;
                    }
                    else doSkillCPD();

                }
                }
                else {
                    if (current_size.x/first_size.x < 0.66f || current_size.y/first_size.y < 0.66f){
                    print("Smaller than the field");
                    re_traj = reproduce_traj(temporal,1.1f);
                    is_skill_cpd = true;
                    doSkillCPD();
                }
                
                else{
                    print("Start adjust");
                    re_traj = reproduce_traj(temporal,1.001f);
                    is_skill_cpd = true;
                    
                    if (current_size.x/first_size.x > 0.95f && current_size.y/first_size.y > 0.95f){
                        print("reproduce finish");
                        end = false;
                    }
                    else doSkillCPD();
                }
            }
                }
                sythesize_skill = cpd_point_list;
            }*/
            temporal.Clear();
            sr.Close();
            fs.Close();
        }
     

    }

    float find_error(List<RVector7> l1, List<RVector7> l2){
        float err = 0;
        int max = 0;
        float sum = 0;
        if (l1.Count < l2.Count) max = l1.Count;
        else max = l2.Count;
        for (int i = 0;i<max;i++){
            sum += dist(l1[i],l2[i]);
        }
        err = sum/max;
        return err;
    }

    List<RVector7> find_frame_points(List<RVector7> list, int num, int mode){
        float min_x = list[0].x;
        float max_x = list[0].x;
        float min_z = list[0].z;
        float max_z = list[0].z;
        float y = list[0].y;
        List<RVector7> frame = new List<RVector7>();
        RVector7 rp1 = new RVector7();
        RVector7 rp2 = new RVector7();
        RVector7 rp3 = new RVector7();
        RVector7 rp4 = new RVector7();
        if (mode == 0 || mode == 2){ //rectangle
            foreach (var item in list){
            if (item.x < min_x) min_x = item.x;
            if (item.x > max_x) max_x = item.x;
            if (item.z < min_z) min_z = item.z;
            if (item.z > max_z) max_z = item.z;
            }
        
        rp1.x = min_x;
        rp1.y = y;
        rp1.z = min_z;
        rp2.x = min_x;
        rp2.y = y;
        rp2.z = max_z;
        rp3.x = max_x;
        rp3.y = y;
        rp3.z = max_z;
        rp4.x = max_x;
        rp4.y = y;
        rp4.z = min_z;
        float len_x = (max_x - min_x)/num;
        float len_z = (max_z - min_z)/num;
        for (int j = 0;j<4;j++){
            if (j == 0) frame.Add(rp1);
            else if (j == 1) frame.Add(rp2);
            else if (j == 2) frame.Add(rp3);
            else frame.Add(rp4);
            for (int i = 0;i<num - 1;i++){
                RVector7 a = new RVector7();
                if (j == 0){
                    a.x = rp1.x;
                    a.y = rp1.y;
                    a.z = rp1.z + len_z;
                    rp1 = a;
                }
                else if (j == 1){
                    a.x = rp2.x + len_x;
                    a.y = rp2.y;
                    a.z = rp2.z;
                    rp2 = a;
                }
                else if (j == 2){
                    a.x = rp3.x;
                    a.y = rp3.y;
                    a.z = rp3.z - len_z;
                    rp3 = a;
                }
                else {
                    a.x = rp4.x - len_x;
                    a.y = rp4.y;
                    a.z = rp4.z;
                    rp4 = a;
                }
                frame.Add(a);
            }
        }
        }
        else if (mode == 1){
            //list.Add(list[0]);
            int idx = 0;
            float len_x, len_y,len_z = 0;
            for (int i = 0; i<4;i++){
                if (i == 3){
                    len_x = (list[0].x - list[i].x)/num;
                    len_y = (list[0].y - list[i].y)/num;
                    len_z = (list[0].z - list[i].z)/num;
                }
                else {
                    len_x = (list[i+1].x - list[i].x)/num;
                    len_y = (list[i+1].y - list[i].y)/num;
                    len_z = (list[i+1].z - list[i].z)/num;
                }
                
                frame.Add(list[i]);
                //print(frame.Count());
                for (int j = 0 ;j<num-1;j++){
                    RVector7 a = new RVector7();
                    
                    a.x = frame[j+idx].x + len_x;
                    a.y = frame[j+idx].y + len_y;
                    a.z = frame[j+idx].z + len_z; 
                    frame.Add(a);
                }
                
                idx = idx + num;
            }
        }
        else if (mode == 3){
            float len_x = (list[3].x - list[0].x)/num;
            float len_y = (list[3].y - list[0].y)/num;
            float len_z = (list[3].z - list[0].z)/num;
                frame.Add(list[0]);
                for (int j = 0 ;j<num;j++){
                    RVector7 a = new RVector7();
                    
                    a.x = frame[j].x + len_x;
                    a.y = frame[j].y + len_y;
                    a.z = frame[j].z + len_z; 
                    frame.Add(a);
                }
                
            
                /*float len_x = (list[2].x - list[1].x)/num;
                float len_y = (list[2].y - list[1].y)/num;
                float len_z = (list[2].z - list[1].z)/num;
                frame.Add(list[1]);
                for (int j = 0 ;j<num;j++){
                    RVector7 a = new RVector7();
                    
                    a.x = frame[j].x + len_x;
                    a.y = frame[j].y + len_y;
                    a.z = frame[j].z + len_z; 
                    frame.Add(a);
                }

                len_x = (list[3].x - list[0].x)/num;
                len_y = (list[3].y - list[0].y)/num;
                len_z = (list[3].z - list[0].z)/num;
                frame.Add(list[0]);
                int i = frame.Count;
                for (int j =i - 1;j<i + num - 1;j++){
                    RVector7 a = new RVector7();
                    
                    a.x = frame[j].x + len_x;
                    a.y = frame[j].y + len_y;
                    a.z = frame[j].z + len_z; 
                    frame.Add(a);
                }*/
        }
        if (mode == 2){
            List<RVector7> points = new List<RVector7>();
            points.Add(rp1);
            points.Add(rp2);
            points.Add(rp3);
            points.Add(rp4);
            return points;
        }
        else
        return frame;
    }

    void reset(){
        /*franka_ref.arm_1 = franka_nativ_state[0];
        franka_ref.arm_2 = franka_nativ_state[1];
        franka_ref.arm_3 = franka_nativ_state[2];
        franka_ref.arm_4 = franka_nativ_state[3];
        franka_ref.arm_5 = franka_nativ_state[4];
        franka_ref.arm_6 = franka_nativ_state[5];
        franka_ref.arm_7 = franka_nativ_state[6];*/

        nmpc_marker.transform.position = new Vector3(initial_nmpc_marker.x, initial_nmpc_marker.y, initial_nmpc_marker.z);
        //is_reproduce_franka = true;
        nmpc_index = 0;
        marker_initial = true;
        robot_point_list.Clear();

    }


    void Start()
    {
        Statics.cpd_manager_ref = this;
        initial_user_point = user.transform.position;
        initial_nmpc_point = nmpc_marker.transform.position;
        franka_nativ_state.Add(franka_ref.arm_1);
        franka_nativ_state.Add(franka_ref.arm_2);
        franka_nativ_state.Add(franka_ref.arm_3);
        franka_nativ_state.Add(franka_ref.arm_4);
        franka_nativ_state.Add(franka_ref.arm_5);
        franka_nativ_state.Add(franka_ref.arm_6);
        franka_nativ_state.Add(franka_ref.arm_7);
        franka_mid_point.x = franka_ref.transform.position.x;
        franka_mid_point.y = point1.transform.position.y;
        franka_mid_point.z = franka_ref.transform.position.z;
        //point.transform.position = new Vector3(franka_mid_point.x,franka_mid_point.y,franka_mid_point.z);
        point_list = new List<RVector7>();
        point_list2 = new List<RVector7>();
        RVector7 tp1 = new RVector7();
        tp1.x = point1.transform.position.x;
        tp1.y = point1.transform.position.y;
        tp1.z = point1.transform.position.z;
        RVector7 tp2 = new RVector7();
        tp2.x = point2.transform.position.x;
        tp2.y = point2.transform.position.y;
        tp2.z = point2.transform.position.z;
        RVector7 tp3 = new RVector7();
        tp3.x = point3.transform.position.x;
        tp3.y = point3.transform.position.y;
        tp3.z = point3.transform.position.z;
        RVector7 tp4 = new RVector7();
        tp4.x = point4.transform.position.x;
        tp4.y = point4.transform.position.y;
        tp4.z = point4.transform.position.z;
        point_list.Add(tp1);
        point_list.Add(tp2);
        point_list.Add(tp3);
        point_list.Add(tp4);
        point_list2.Add(tp1);
        point_list2.Add(tp2);
        point_list2.Add(tp3);
        point_list2.Add(tp4);
        //double uplimit[7] = {2.87, 1.74, 2.87, -0.06, 2.87, 3.6, 2.87};
        //double lowlimit[7] = {-2.87, -1.74, -2.87, -3.05, -2.87, -0.014, -2.87};
        //print(franka_ref.arm_+'1');
        
        //print(nmpc_marker.transform.localPosition);
        resetPoint();
    }

    public void Main_tele_network_eventDataUpdated()
    {
        tele_robot_location = Statics.main_tele_network.get;
        //print(tele_robot_location.x);
    }
    public void Main_cpd_network_eventDataUpdated()
    {
        print("Get the CPD result back!");

        cpd_point_list = new List<RVector7>();

        RRSCPDResult cmd = Statics.main_cpd_network.get;
        
        if (cmd.result_iterations != 0 )
        {
            print("Len: " + cmd.result_points.Length);
            print("Sigma2 " + cmd.result_2);
            print("Scale " + cmd.result_1);
            print("Iterations" + cmd.result_iterations);
            cpdscale = cmd.result_1;


            for (int i = 0; i < cmd.result_points.Length; i++)
            {
                cpd_point_list.Add(cmd.result_points[i]);

                //Reput the qs to registered points
                
                   // cpd_point_list[i].qx = skill_point_list[i].qx;
                   // cpd_point_list[i].qy = skill_point_list[i].qy;
                   // cpd_point_list[i].qz = skill_point_list[i].qz;
                   // cpd_point_list[i].qw = skill_point_list[i].qw;
                
                
            }

            update_render_cpd_result = true;
        }
        

        
        time_cpd_request_done = DateTime.Now;
        print("Time is ");
        print("CPD Registeration Time : " + (time_cpd_request_done - time_cpd_request_start).TotalMilliseconds.ToString() + " ms");
        cpd_valid_result = true;
    }

    float dist(RVector7 a, RVector7 b)
    {
        float d = Mathf.Sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
        return d;
    }

    float dist_xz(RVector7 a, Vector3 b)
    {
        float d = Mathf.Sqrt((a.x - b.x) * (a.x - b.x) + (a.z - b.z) * (a.z - b.z));
        return d;
    }

    float dist_xy(RVector7 a, Vector3 b)
    {
        float d = Mathf.Sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
        return d;
    }

    float dist_xz_points(RVector7 a, RVector7 b)
    {
        float d = Mathf.Sqrt((a.x - b.x) * (a.x - b.x) + (a.z - b.z) * (a.z - b.z));
        return d;
    }


    float findClosestDistinSkill(GameObject point, List<GameObject> list)
    {
        int min_index = 0;
        float min_distance = 100000000;

        int start = 0;
        int finish = list.Count;

        for (int index = start; index < finish; index++)
        {
            var item = list[index];
            float distance = Mathf.Sqrt((item.transform.position.x - point.transform.position.x) * (item.transform.position.x - point.transform.position.x) + (item.transform.position.y - point.transform.position.y) * (item.transform.position.y - point.transform.position.y) + (item.transform.position.z - point.transform.position.z) * (item.transform.position.z - point.transform.position.z));

            if (distance < min_distance)
            {
                min_distance = distance;
                min_index = index;
            }
        }

        return min_distance;
    }

    int findClosestStepinSkill(RVector7 point, List<RVector7> list, int current_predicted_step = -1)
    {
        int min_index = 0;
        float min_distance = 100000000;

        int start = 0;
        int finish = list.Count;

        if (current_predicted_step != -1)
        {
            start = current_predicted_step - 20;
            finish = current_predicted_step + 20;
        }

        if (start < 0) start = 0;
        if (finish > list.Count) finish = list.Count;

        for (int index = start; index < finish; index++)
        {
            var item = list[index];
            float distance = Mathf.Sqrt((item.x - point.x) * (item.x - point.x) + (item.y - point.y) * (item.y - point.y) + (item.z - point.z) * (item.z - point.z));

            if (distance < min_distance)
            {
                min_distance = distance;
                min_index = index;
            }
        }

        return min_index;
    }

    float findClosestDistanceinSkill(RVector7 point, List<RVector7> list)
    {
        int min_index = 0;
        float min_distance = 100000000;

        int start = 0;
        int finish = list.Count;

        if (start < 0) start = 0;
        if (finish > list.Count) finish = list.Count;

        for (int index = start; index < finish; index++)
        {
            var item = list[index];
            float distance = Mathf.Sqrt((item.x - point.x) * (item.x - point.x) + (item.y - point.y) * (item.y - point.y) + (item.z - point.z) * (item.z - point.z));

            if (distance < min_distance)
            {
                min_distance = distance;
                min_index = index;
            }
        }

        return min_index;
    }
    
    void KeyE(){
        print("Eliminate the markers.");
        if (nmpc_marker_list.Count != skill_point_list.Count)
            print("Not the marker list.");
        else {
            List<RVector7> temp = new List<RVector7>();
            for (int i = 0;i<nmpc_marker_list.Count;i++){
                if (i%2==0)
                    temp.Add(nmpc_marker_list[i]);
            }
            nmpc_marker_list = temp;
            foreach (var item in cpd_point_object_list)
            {
                Destroy(item);
            }
            foreach (var item in robot_point_object_list)
            {
                Destroy(item);
            }
            foreach (var item in skill_point_object_list)
            {
                Destroy(item);
            }
            foreach (var item in user_point_object_list)
            {
                Destroy(item);
            }
            foreach (var item in nmpc_marker_object_list)
            {
                Destroy(item);
            }
            nmpc_marker_object_list = new List<GameObject>();
            foreach (var item in nmpc_marker_list)
            {
                GameObject g = Instantiate(skillpoint_prefab, Vector3.zero, Quaternion.identity);

                g.transform.localPosition = new Vector3(item.x, item.y, item.z);

                nmpc_marker_object_list.Add(g);
            }

        }
    }

    void KeyB()
    {
        reset_trail = 1;
        print("Begin Experiments");
        robot_point_list = new List<RVector7>();
        is_begin = true;
        is_teleoperation_mode = true;
    }

    void KeyI(){
        skill_file_list.Add("12.txt");
        skill_file_list.Add("13.txt");
        skill_file_list.Add("20.txt");
        sythesize_new_skill(skill_file_list);
        is_synthesize = true;
        finish_synthesize = false;
        process_mode = 3;
    }

    void KeyL()
    {
        /*RVector7 pp = new RVector7();
        pp.x = point.transform.position.x;
        pp.y = point.transform.position.y;
        pp.z = point.transform.position.z;
        //print(find_point(point1.transform.position,point2.transform.position,point3.transform.position,point4.transform.position,pp,XorZ));
        //print(find_angle(point1.transform.position,point2.transform.position,point3.transform.position));
        print("Load the recorded skill");*/
        List<RVector7> aaa = new List<RVector7>();
        
        /*aaa = fill_skill(find_frame_points(point_list,40,2),40,40);
        print(aaa.Count());
        nmpc_marker_object_list = new List<GameObject>();

            foreach (var item in cpd_point_list)
            {
                GameObject g = Instantiate(cpdpoint_prefab, Vector3.zero, Quaternion.identity);

                g.transform.localPosition = new Vector3(item.x, item.y, item.z);

                cpd_point_object_list.Add(g);
            }

            foreach (var item in aaa)
            {
                GameObject g = Instantiate(cpdpoint_prefab, Vector3.zero, Quaternion.identity);

                g.transform.localPosition = new Vector3(item.x, item.y, item.z);

                nmpc_marker_object_list.Add(g);
            }*/

        skill_point_list = new List<RVector7>();
        //nmpc_marker_list = new List<RVector7>();

        FileStream fs = new FileStream(Application.dataPath + "/Benchmarks/skills/" + skill_file_name, FileMode.Open, FileAccess.Read);
        StreamReader sr = new StreamReader(fs);

        string a = sr.ReadToEnd();
        char[] ccc = new char[1];
        ccc[0] = '\n';

        char[] ccc2 = new char[1];
        ccc2[0] = ',';

        string[] list = a.Split(ccc);

        try
        {
            foreach (var item in list)
            {
                if (item == "") continue;

                string[] items = item.Replace("\r", string.Empty).Split(ccc2);

                RVector7 point = new RVector7();
                point.x = float.Parse(items[0]);
                point.y = float.Parse(items[1]);
                point.z = float.Parse(items[2]);

                point.qx = float.Parse(items[3]);
                point.qy = float.Parse(items[4]);
                point.qz = float.Parse(items[5]);
                point.qw = float.Parse(items[6]);

                skill_point_list.Add(point);
            }
        }
        catch (Exception ee)
        {
            string me = ee.Message;
        }

        print("load skill with " + skill_point_list.Count + " points");

        sr.Close();
        fs.Close();

        //skill_file_list.Add("12.txt");
        //skill_file_list.Add("13.txt");
        //skill_file_list.Add("20.txt");
        
    
        foreach (var item in skill_point_object_list)
        {
            Destroy(item);
        }

        skill_point_object_list = new List<GameObject>();
        nmpc_marker_object_list = new List<GameObject>();

        
        

        //print(find_lwh(skill_point_list,0).x);
        //print(fill_skill(skill_point_list,40,40).Count);
        finish_synthesize = true;
         if (skill_point_list2.Count==0){
                foreach (var item in skill_point_list){
                    RVector7 i = new RVector7();
                    i.x = item.x;
                    i.y = item.z;
                    i.z = item.y;
                    skill_point_list2.Add(i);
                }
                

            }
            List<RVector7> bbb = new List<RVector7>();
            bbb = find_frame_points(point_list2,10,1);
            
        aaa = find_skill_frame(skill_point_list2,20);
        //print(aaa.Count);
        print(bbb.Count);
        foreach (var item in skill_point_list2){
            aaa.Add(item);
        }
        /*RVector7 ab = new RVector7();
        ab.x = nmpc_marker.transform.position.x;
        ab.y = nmpc_marker.transform.position.y;
        ab.z = nmpc_marker.transform.position.z;
        convert2nmpc(skill_point_list2,ab);
        int ii = 0;
        foreach (var item in nmpc_marker_list){
            if (ii < 20) aaa.Add(item);
            ii++;
            if (ii == 20){
                aaa.Add(item);
                aaa.Add(item);
                aaa.Add(item);
                aaa.Add(item);
                aaa.Add(item);
            }

        }*/
        foreach (var item in aaa)
        {
            GameObject g = Instantiate(skillpoint_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
            skill_point_object_list.Add(g);
        }
        /*foreach (var item in skill_point_list)
        {
        GameObject g = Instantiate(skillpoint_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
            nmpc_marker_object_list.Add(g);
        }*/
    }

    void KeyX(){
        print("point-based robot register.");
        iter = 0;
        process_mode = 8;
        RVector7 sp = new RVector7();
        sp.x = nmpc_marker.transform.position.x;
        sp.y = nmpc_marker.transform.position.y;
        sp.z = nmpc_marker.transform.position.z;
            
        initial_nmpc_marker.x = nmpc_marker.transform.position.x;
        initial_nmpc_marker.y = nmpc_marker.transform.position.y;
        initial_nmpc_marker.z = nmpc_marker.transform.position.z; 
        convert2nmpc(skill_point_list2,sp);
        step_time = Time.time;
    }

    void KeyM(){
        //if (process_mode != 1 && process_mode != 9){
            //process_mode = 1;
           // print("Check all points.");
        //}
       // else if (process_mode == 1){

            process_mode = 9;
            print("Check the limit points.");
            RVector7 sp = new RVector7();
        sp.x = nmpc_marker.transform.position.x;
        sp.y = nmpc_marker.transform.position.y;
        sp.z = nmpc_marker.transform.position.z;
            
        initial_nmpc_marker.x = nmpc_marker.transform.position.x;
        initial_nmpc_marker.y = nmpc_marker.transform.position.y;
        initial_nmpc_marker.z = nmpc_marker.transform.position.z; 
        convert2nmpc(skill_point_list2,sp);
        step_time = Time.time;
        iter = 0;
        skill_point_object_list = new List<GameObject>();
        foreach (var item in nmpc_marker_list)
        {
            GameObject g = Instantiate(skillpoint_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
            skill_point_object_list.Add(g);
        }
        //}
        //else if (process_mode == 9){
            //process_mode = 1;
           // print("Check all points.");
        //}
    }

    

    void KeyN(){
        /*if (!is_nmpc){
            is_nmpc = true;
            print("NMPC started."); 
        }
        else {
            is_nmpc = false;
            print("NMPC stopded.");
        }
        
        //process_mode = 1; 
        is_reproduce_franka = true;
        initial_nmpc_marker.x = nmpc_marker_list[0].x;
        initial_nmpc_marker.y = nmpc_marker_list[0].y;
        initial_nmpc_marker.z = nmpc_marker_list[0].z; */
         process_mode = 1;
            print("Plane-based robot registration.");
            
        step_time = Time.time;
        iter = 0;
        
        
    }

    void KeyK(){
        is_plane_register = true;
        process_mode = 2;
        step_time = Time.time;
       
       

    }
    
    void KeyJ(){
        print("Plane register once CPD");
        is_plane_register2 = true;
        process_mode = 100;
        step_time = Time.time;
         
    }

    void KeyH(){
        print("point-based plane register.");
        is_plane_register3 = true;
        process_mode = 200;
        step_time = Time.time;
        //is_plane_register = true;
        link_limit = whiteboard.transform.position.x - 5*whiteboard.transform.localScale.x;
        right_limit = whiteboard.transform.position.x + 5*whiteboard.transform.localScale.x;
        up_limit = whiteboard.transform.position.z + 5*whiteboard.transform.localScale.z;
        down_limit = whiteboard.transform.position.z - 5*whiteboard.transform.localScale.z;
    }

    void KeyO(){
        if (back_to_initial_plane){
            back_to_initial_plane = false;
            process_mode = 4;
            print("Back to initial plane.");
        }
        else {
                step_time = Time.time;
                process_mode = 5;
                is_rotated = true;
                print("Default - Based on the midpoint.");
            
            
        }
        
        /*if (is_rotated){
            
            Vector3 plane_mid_point = new Vector3();
            plane_mid_point = find_midpoint(point_list);
            float di = dist_xz(franka_mid_point,plane_mid_point);
            float dx = plane_mid_point.x - franka_mid_point.x;
            float dz = plane_mid_point.z - franka_mid_point.z;
            angle = Mathf.Atan(dx/dz);
            if (dz < 0) angle = 3.14f - angle;
            print(angle);
        
            float rx = (point_list[0].x - franka_mid_point.x)*Mathf.Cos(angle) - (point_list[0].z - franka_mid_point.z)*Mathf.Sin(angle) + franka_mid_point.x;
            float rz = (point_list[0].x - franka_mid_point.x)*Mathf.Sin(angle) + (point_list[0].z - franka_mid_point.z)*Mathf.Cos(angle) + franka_mid_point.z;
            point1.transform.position = new Vector3(rx,plane_mid_point.y,rz);

            rx = (point_list[1].x - franka_mid_point.x)*Mathf.Cos(angle) - (point_list[1].z - franka_mid_point.z)*Mathf.Sin(angle) + franka_mid_point.x;
            rz = (point_list[1].x - franka_mid_point.x)*Mathf.Sin(angle) + (point_list[1].z - franka_mid_point.z)*Mathf.Cos(angle) + franka_mid_point.z;
            point2.transform.position = new Vector3(rx,plane_mid_point.y,rz);

            rx = (point_list[2].x - franka_mid_point.x)*Mathf.Cos(angle) - (point_list[2].z - franka_mid_point.z)*Mathf.Sin(angle) + franka_mid_point.x;
            rz = (point_list[2].x - franka_mid_point.x)*Mathf.Sin(angle) + (point_list[2].z - franka_mid_point.z)*Mathf.Cos(angle) + franka_mid_point.z;
            point3.transform.position = new Vector3(rx,plane_mid_point.y,rz);

            rx = (point_list[3].x - franka_mid_point.x)*Mathf.Cos(angle) - (point_list[3].z - franka_mid_point.z)*Mathf.Sin(angle) + franka_mid_point.x;
            rz = (point_list[3].x - franka_mid_point.x)*Mathf.Sin(angle) + (point_list[3].z - franka_mid_point.z)*Mathf.Cos(angle) + franka_mid_point.z;
            point4.transform.position = new Vector3(rx,plane_mid_point.y,rz);
            point_list.Clear();
            RVector7 tp1 = new RVector7();
            tp1.x = point1.transform.position.x;
            tp1.y = point1.transform.position.y;
            tp1.z = point1.transform.position.z;
            RVector7 tp2 = new RVector7();
            tp2.x = point2.transform.position.x;
            tp2.y = point2.transform.position.y;
            tp2.z = point2.transform.position.z;
            RVector7 tp3 = new RVector7();
            tp3.x = point3.transform.position.x;
            tp3.y = point3.transform.position.y;
            tp3.z = point3.transform.position.z;
            RVector7 tp4 = new RVector7();
            tp4.x = point4.transform.position.x;
            tp4.y = point4.transform.position.y;
            tp4.z = point4.transform.position.z;
            point_list.Add(tp1);
            point_list.Add(tp2);
            point_list.Add(tp3);
            point_list.Add(tp4);
        }
        else{
            process_mode = 4;
            print(point_list2[0].x);
        }*/
        
    }

    void KeyD(){
        if (is_register_3D){
            print("Switch to 2D register.");
            is_register_3D = false;
        }
        else {
            print("Switch to 3D register.");
            is_register_3D = true;
        }

        
    }

    void KeyT()
    {
        /*is_teleoperation_mode = !is_teleoperation_mode;

        if (is_teleoperation_mode)
        {
            print("Network Connected");

            is_teleoperation_mode = true;
            is_switch_to_skill = false;
        }
        else
        {
            print("Network Disconneted");

            if (operation_mpde == Mode.SharedAutonomyCPD)
            {
                cpd_valid_result = false;
                doSkillCPD();
                
            }

            is_teleoperation_mode = false;
            is_switch_to_skill = true;
            is_go_to_skill = true;
        }

        resetPoint();*/
        process_mode = 7;
    }

    void KeyC(){
        
        is_convex_plane = true;
        if (is_random_plane){
            is_random_plane = false;
            print("Add random plane.");
        }
        else {
            is_random_plane = true;
            print("Add convex plane.");
        }
        if (point_list.Count == 0){
            
            RVector7 tp1 = new RVector7();
            tp1.x = point1.transform.position.x;
            tp1.y = point1.transform.position.y;
            tp1.z = point1.transform.position.z;
            RVector7 tp2 = new RVector7();
            tp2.x = point2.transform.position.x;
            tp2.y = point2.transform.position.y;
            tp2.z = point2.transform.position.z;
            RVector7 tp3 = new RVector7();
            tp3.x = point3.transform.position.x;
            tp3.y = point3.transform.position.y;
            tp3.z = point3.transform.position.z;
            RVector7 tp4 = new RVector7();
            tp4.x = point4.transform.position.x;
            tp4.y = point4.transform.position.y;
            tp4.z = point4.transform.position.z;
            point_list.Add(tp1);
            point_list.Add(tp2);
            point_list.Add(tp3);
            point_list.Add(tp4);
        }
        convex_plane = add_points(point_list,20);
        //print(convex_plane.Count);
        List<float> differ_y = new List<float>();
        float aa = -0.00625f * Random.Range(1,5);
        float bb = 0.125f * Random.Range(1,5);
        for (int i = 0;i<Mathf.Sqrt(convex_plane.Count);i++){
            float a = aa * i * i + bb * i;
            differ_y.Add(a);
        }
        float[] scale_list1 = new float[10];//for random plane
        float[] scale_list2 = new float[10];//for convex plane
        float ss1 = -0.5f;
        float ss2 = 0.95f; 
        for (int i = 0; i < 10;i++){
            scale_list1[i] = ss1;
            scale_list2[i] = ss2;
            ss1 += 0.1f;
            ss2 += 0.01f;
        }
        
        //int n = Random.Range(0,9);
        if (is_random_plane){
            foreach (var item1 in convex_plane){
            item1.y += scale_list1[Random.Range(0,9)];
            
        }
        }
        else {
            int j = 0;
            foreach (var item1 in convex_plane){
            item1.y += differ_y[j] * scale_list2[Random.Range(0,9)];
            j++;
            if (j == differ_y.Count) {
                j = 0;
            }
        }
        }
        foreach (var item in user_point_object_list)
        {
            Destroy(item);
        }
        

        user_point_object_list = new List<GameObject>();
        

        foreach (var item in convex_plane)
        {
            GameObject g = Instantiate(skillpoint_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
            user_point_object_list.Add(g);
        }
        
    }


    void KeyP()
    {
        reset_trail = 1;

        robot_point_object_list = new List<GameObject>();
        nmpc_marker_object_list = new List<GameObject>();

        foreach (var item in nmpc_marker_list)
        {
            GameObject g = Instantiate(nmpc_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
            nmpc_marker_object_list.Add(g);
        }

        foreach (var item in robot_point_list)
        {
            GameObject g = Instantiate(nmpc_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
            robot_point_object_list.Add(g);
        }

        user_point_object_list = new List<GameObject>();

        foreach (var item in user_point_list)
        {
            GameObject g = Instantiate(plane_prefab, new Vector3(item.gx, item.gy, item.gz), Quaternion.identity);
            user_point_object_list.Add(g);
        }

        foreach (var item in skill_point_object_list)
        {
            Destroy(item);
        }

        skill_point_object_list = new List<GameObject>();

        foreach (var item in cpd_point_object_list)
        {
            Destroy(item);
        }

        cpd_point_object_list = new List<GameObject>();

        //
        p_max_distance = calculate_max_delta_d();
        p_tracking_error = calculate_user_tracking_error();
        p_user_traj_len = calculate_user_traj_length();
        p_robot_traj_len = calculate_robot_traj_length();

        print("max_distance = " + p_max_distance.ToString());
        print("total_tracking_error = " + p_tracking_error.ToString());
        print("user_traj_len = " + p_user_traj_len.ToString());
        print("robot_traj_len = " + p_robot_traj_len.ToString());

        print("User points " + user_point_list.Count.ToString());
        print("Robot points " + robot_point_list.Count.ToString());
    }

    void FixedUpdate()
    {   if (franka_ref.env == "real") eef_offset = -1;
        else eef_offset = 1;

        if (process_mode == 5 && is_rotated){//convert the plane based on 1-4 boundary 
            
            //process_mode = 7;
            //print(dist(point_list2[0],point_list2[3]));
            //print(dist(point_list2[1],point_list2[2]));
            //print(dist(point_list2[2],point_list2[3]));
            //print(dist(point_list2[0],point_list2[1]));
            point1.transform.position = new Vector3(point.transform.position.x,point.transform.position.y,point.transform.position.z);
            float d14 = dist(point_list[0],point_list[3]);
            point4.transform.position = new Vector3(point.transform.position.x + d14,point.transform.position.y,point.transform.position.z);
            Vector3 vector12,vector14,vector43,vector41 = new Vector3();
            vector12.x = point_list[1].x - point_list[0].x;
            vector12.y = point_list[1].y - point_list[0].y;
            vector12.z = point_list[1].z - point_list[0].z;
            vector14.x = point_list[3].x - point_list[0].x;
            vector14.y = point_list[3].y - point_list[0].y;
            vector14.z = point_list[3].z - point_list[0].z;
            vector43.x = point_list[2].x - point_list[3].x;
            vector43.y = point_list[2].y - point_list[3].y;
            vector43.z = point_list[2].z - point_list[3].z;
            vector41.x = point_list[0].x - point_list[3].x;
            vector41.y = point_list[0].y - point_list[3].y;
            vector41.z = point_list[0].z - point_list[3].z;
            float cosa214 = (vector12.x * vector14.x + vector12.y * vector14.y + vector12.z * vector14.z)/
            (Mathf.Sqrt(vector12.x * vector12.x + vector12.y * vector12.y + vector12.z * vector12.z)*Mathf.Sqrt(vector14.x * vector14.x + vector14.y * vector14.y + vector14.z * vector14.z));
            float cosa341 = (vector43.x * vector41.x + vector43.y * vector41.y + vector43.z * vector41.z)/
            (Mathf.Sqrt(vector43.x * vector43.x + vector43.y * vector43.y + vector43.z * vector43.z)*Mathf.Sqrt(vector41.x * vector41.x + vector41.y * vector41.y + vector41.z * vector41.z));
            float d12 = dist(point_list[0],point_list[1]);
            float d43 = dist(point_list[3],point_list[2]);
            float d12x = d12 * cosa214;
            float d12z = Mathf.Sqrt(d12 * d12 - d12x * d12x);
            if (cosa214 > 0)
                point2.transform.position = new Vector3(point.transform.position.x + d12x,point.transform.position.y,point.transform.position.z + d12z);
            
            else 
                point2.transform.position = new Vector3(point.transform.position.x - d12x,point.transform.position.y,point.transform.position.z + d12z);
            float d43x = d43 * cosa341;
            float d43z = Mathf.Sqrt(d43 * d43 - d43x * d43x);
            if (cosa341 > 0)
                point3.transform.position = new Vector3(point4.transform.position.x - d43x,point4.transform.position.y,point4.transform.position.z + d43z);
            
            else 
                point3.transform.position = new Vector3(point4.transform.position.x + d43x,point4.transform.position.y,point4.transform.position.z + d43z);
            point_list.Clear();
            RVector7 tp1 = new RVector7();
            tp1.x = point1.transform.position.x;
            tp1.y = point1.transform.position.y;
            tp1.z = point1.transform.position.z;
            RVector7 tp2 = new RVector7();
            tp2.x = point2.transform.position.x;
            tp2.y = point2.transform.position.y;
            tp2.z = point2.transform.position.z;
            RVector7 tp3 = new RVector7();
            tp3.x = point3.transform.position.x;
            tp3.y = point3.transform.position.y;
            tp3.z = point3.transform.position.z;
            RVector7 tp4 = new RVector7();
            tp4.x = point4.transform.position.x;
            tp4.y = point4.transform.position.y;
            tp4.z = point4.transform.position.z;
            point_list.Add(tp1);
            point_list.Add(tp2);
            point_list.Add(tp3);
            point_list.Add(tp4);
            //print(dist(point_list[0],point_list[3]));
            //print(dist(point_list[1],point_list[2]));

            //print(dist(point_list[2],point_list[3]));
            //print(dist(point_list[0],point_list[1]));
            back_to_initial_plane = true;
            
            
        //is_rigid = false;
        //doSkillCPD();
        }

        //if (process_mode == 5 && is_rotated){}

        if (process_mode == 4 && !cpd_valid_result){
            
            process_mode = 40;
            if (nmpc_marker_list.Count != 0){
                print("Register the skill to old plane.");
                new_skill = new List<RVector7>();
                reproduce_list = new List<RVector7>();
                
                new_skill = find_frame_points(point_list,50,3);
                reproduce_list = find_frame_points(point_list2,50,3);
                //new_skill = add_points(point_list,10);
                //reproduce_list = add_points(point_list2,10);
                
                
                foreach  (var item in nmpc_marker_list){
                    new_skill.Add(item);
                }
                new_skill.Add(point_list[1]);
                new_skill.Add(point_list[2]);
                print(new_skill.Count);
              /* robot_point_object_list = new List<GameObject>();
        nmpc_marker_object_list = new List<GameObject>();

        foreach (var item in new_skill)
        {
            GameObject g = Instantiate(nmpc_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
            nmpc_marker_object_list.Add(g);
        }

        foreach (var item in reproduce_list)
        {
            GameObject g = Instantiate(nmpc_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
            robot_point_object_list.Add(g);
        }*/
                point1.transform.position = new Vector3(point_list2[0].x,point_list2[0].y,point_list2[0].z);
                point2.transform.position = new Vector3(point_list2[1].x,point_list2[1].y,point_list2[1].z);
                point3.transform.position = new Vector3(point_list2[2].x,point_list2[2].y,point_list2[2].z);
                point4.transform.position = new Vector3(point_list2[3].x,point_list2[3].y,point_list2[3].z);
                doSkillCPD();
                                

            }
            else print("No skill.");
        }

        if (process_mode == 41 && cpd_valid_result){
                cpd_valid_result = false;
                process_mode = 42;
                new_skill = new List<RVector7>();
                reproduce_list = new List<RVector7>();
                //new_skill = cpd_point_list;
                //reproduce_list = find_frame_points(point_list2,50,1);
                for (int q = 0; q < 5; q++){
                    new_skill.Add(cpd_point_list[cpd_point_list.Count - 2]);
                    new_skill.Add(cpd_point_list[cpd_point_list.Count - 1]);
                    reproduce_list.Add(point_list2[1]);
                    reproduce_list.Add(point_list2[2]);
                }
                int j = 0;
                foreach (var item in cpd_point_list){
                    j++;
                    if (j <= 21){
                        new_skill.Add(item);
                        reproduce_list.Add(item);
                    }
                    if (j > 21 && j < cpd_point_list.Count - 1)
                        new_skill.Add(item);
                }
                doSkillCPD();
                
                /*nmpc_marker_list.Clear();
                int i = 0;
                foreach (var item in cpd_point_list){
                    i++;
                    if (i > 42) nmpc_marker_list.Add(item);
                }
                print(nmpc_marker_list.Count);
                robot_point_object_list = new List<GameObject>();
                foreach (var item in reproduce_list){
                    GameObject g = Instantiate(skillpoint_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
                    robot_point_object_list.Add(g);
                }*/

            
        }

        if (process_mode == 43 && cpd_valid_result){
            cpd_valid_result = false;
            is_rigid = true;
            print("Extract the NMPC points.");
            if (bench_state == 10) bench_state = 2;
            
            nmpc_marker_list.Clear();
                int i = 0;
                foreach (var item in cpd_point_list){
                    i++;
                    if (i > 61) nmpc_marker_list.Add(item);
                }
                print(nmpc_marker_list.Count);
                
                skill_point_list2 = nmpc_marker_list;
                nmpc_marker.transform.position = new Vector3(nmpc_marker_list[0].x,nmpc_marker_list[0].y,nmpc_marker_list[0].z);
                //nmpc_marker.transform.rotation = point1.transform.rotation;
                user_point_object_list = new List<GameObject>();
                //foreach (var item in skill_point_list2){
                 //   GameObject g = Instantiate(plane_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
                  //  user_point_object_list.Add(g);
               // }
                last_time = Time.time - step_time;
                print("finish."+last_time+"rigid CPD times: "+max_step +"non-rigid times: " + max_step_non);
        }

        if (cpd_valid_result && process_mode == 3) is_synthesize = true;
        if (is_synthesize){
            print("Synthesized skill analyse");
            is_synthesize = false;
            cpd_valid_result = false;
            //print(size_list.Count);
            //print(count_list[0]);
            //print(count_list[1]);
            //print(count_list[2]);
            List<RVector7> cut_skill_list = new List<RVector7>();
            bool large_or_small = false;
            
            Vector3 current_size = new Vector3();
            Vector3 first_size = size_list[0];
            Vector3 offset = new Vector3();
            print(first_size.y);
            if (current_idx == 0){
                print("load first skill");
                
                foreach (var item in sythesize_skill)
                {
                    skill_point_list.Add(item);
                    start_idx++;
                    
                    
                    if (start_idx >= count_list[current_idx])
                    {
                        break;
                    }
                }
                is_synthesize = true;
                //print(skill_point_list.Count);
                current_idx++;
            }
            else if(current_idx > 0 && current_idx < count_list.Count){
                
                //start_idx += 1;
                
                //print(cut_skill_list.Count);
                
                if (cpd_point_list.Count == 0){
                    //skill_point_list2 = cut_skill_list;
                    current_size = size_list[current_idx];
                    int i = 0;
                    int j = start_idx;
                    print("load next skill");
                    foreach (var item in sythesize_skill)
                {
                    if (i < j){
                        i++;
                    }
                    else {
                        cpd_point_list.Add(item);
                        skill_point_list2.Add(item);
                        start_idx++;
                    }
                    

                    if (start_idx >= j + count_list[current_idx])
                    {
                        break;
                    }
                }
                    if ( current_size.y/first_size.y > 1)large_or_small = true;//skill larger

                }
                else {
                    //cut_skill_list = cpd_point_list;
                    current_size = new Vector3();
                    current_size = find_lwh(cpd_point_list,0,1);
                    print("change size");
                }

                
                bool end = false;
                print(current_size.y/first_size.y);
                if (large_or_small){
                    if ( current_size.y/first_size.y > 1.1f){
                        print("Larger than the first letter");
                        
                        re_traj = reproduce_traj(cpd_point_list,0.9f);
                        is_skill_cpd = true;
                        doSkillCPD();
                        
                    }
                    else{
                    
                        print("finish");
                        end = true;
                   

                }
                }
                else {
                    if (current_size.y/first_size.y < 0.9f){
                    print("Smaller than the field");
                    print(cpd_point_list.Count);
                    re_traj = reproduce_traj(cpd_point_list,1.1f);
                    is_skill_cpd = true;
                    doSkillCPD();
                    
                }
                
                else{
                    print("reproduce finish");
                    end = true;                   
                }
            }
                if (end){
                    
                    offset = find_offset(skill_point_list,cpd_point_list);
                    foreach (var item1 in cpd_point_list){
                        item1.x += offset.x;
                        item1.y += offset.y;
                        skill_point_list.Add(item1);
                        
                    }
                    current_idx++; 
                    print(skill_point_list.Count);
                    skill_point_list2.Clear();
                    cpd_point_list.Clear();
                    if (current_idx < count_list.Count)
                    is_synthesize = true;
                    foreach (var item in skill_point_object_list)
            {
                Destroy(item);
            }
            foreach (var item in cpd_point_object_list)
            {
                Destroy(item);
            }

            skill_point_object_list = new List<GameObject>();
        //nmpc_marker_object_list = new List<GameObject>();
            print("Skill size: "+skill_point_list.Count);
            foreach (var item in skill_point_list)
            {
                GameObject g = Instantiate(skillpoint_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
                skill_point_object_list.Add(g);
            }
                         
                        }
                               
            }

        }
        if ((process_mode == 20 || process_mode == 28) && cpd_valid_result){//check the cpd result, if the dist of points is too large, redo cpd 
                    print("Check the skill.");
                    float reproduce_list_min = 0;
                    float d_cpd_skill = 0;
                    float dc = 0;
                    float average_d = 0;
                    float max_d = -9999;
                    float min_d = 9999;
                    cpd_valid_result = false;
                    bool redo_cpd = false;
                    int th = 0;
                    if (skill_point_list.Count > 100) th = 3;
                    else th = 2;
                    float err = find_error(reproduce_list,cpd_point_list);
                        print("The minimal average is "+err);
                    /*for (int i = 0;i<cpd_point_list.Count - skill_point_list.Count - 2;i++){
                         dc = dist(reproduce_list[i],cpd_point_list[i]);
                         //if (dc > max_d) max_d = dc;
                         //if (dc < min_d) min_d = dc;
                           // dc = dist(reproduce_list[i],cpd_point_list[i]);
                            if (dc > 10) {
                                average_d = 99999;
                                break;
                            }
                            average_d += dc;
                        
                    }
                    print(average_d);*/
                    if (err < average_min) {
                        average_min = err;
                        cpd_point_list_min = cpd_point_list;
                        reproduce_list_min = reproduce_list.Count;
                        //print(cpd_point_list_min[99].x);
                        //print(cpd_point_list_min[99].y);
                        //print(cpd_point_list_min[99].z);
                    }
                    
                    //redo_cpd = true;
                    //average_d = average_d / (cpd_point_list.Count - skill_point_list.Count);
                    //if (max_d > 100 * min_d) redo_cpd = true;
                    if (nonrigid_parameter > th && !is_register_3D){
                        nonrigid_parameter--;
                        
                        if (process_mode == 20){
                            
                            new_skill = find_frame_points(point_list,Mathf.FloorToInt(skill_point_list.Count/nonrigid_parameter),0);
                            reproduce_list = find_frame_points(point_list,Mathf.FloorToInt(skill_point_list.Count/nonrigid_parameter),1);
                            foreach (var item1 in nmpc_marker_list){
                                new_skill.Add(item1);
                            }
                        
                        
                        }
                        if (process_mode == 28){
                            new_skill = find_skill_frame(skill_point_list2,skill_point_list.Count/nonrigid_parameter);
                            reproduce_list = find_frame_points(point_list,skill_point_list.Count/nonrigid_parameter,1);
                            foreach (var item1 in skill_point_list2){
                                new_skill.Add(item1);                        
                            }
                            process_mode = 29;
                        }
                        print("Redo the CPD.");
                        doSkillCPD();
                        

                    }
                    else {
                        print("The minimal average is "+ average_min);
                        process_mode = 21;
                        is_rigid = true;
                        nonrigid_parameter = 5;
                        average_min = 9999;
                        cpd_point_list = cpd_point_list_min;
                        //extract skill
                        int j = 0;
                        skill_point_list2.Clear();
                        foreach (var item in cpd_point_list){
                            if (j>=reproduce_list_min)
                                skill_point_list2.Add(item);
                            j++;
                        }
                        if (!back_to_initial_plane){
                            nmpc_marker.transform.position = new Vector3(skill_point_list2[0].x, skill_point_list2[0].y, skill_point_list2[0].z);
                        }
                        
                        nmpc_marker_object_list = new List<GameObject>();
                        foreach (var item in nmpc_marker_list){
                            GameObject g = Instantiate(skillpoint_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
                            nmpc_marker_object_list.Add(g);
                        }
                        //print(cpd_point_list_min[99].x);
                        //print(cpd_point_list_min[99].y);
                        //print(cpd_point_list_min[99].z);
                    }
        }

        if (process_mode == 21 && finish_plane_register) {
            finish_plane_register = false;
            
                cpd_valid_result = false;
                print("Genrate NMPC marker list.");
                cpd_valid_result = false;
                int i = cpd_point_list.Count - skill_point_list.Count;
                int j = 0;
                nmpc_marker_list.Clear();
                foreach (var item in cpd_point_list){
                    j++;
                    if (j > i){
                        nmpc_marker_list.Add(item);
                    }

                }
                if (is_convex_plane && !is_register_3D){
                    is_convex_plane = false;
                    print("Register to plane.");
                    float d;
                    
                    foreach (var item in nmpc_marker_list){
                        float dmin = 99999;
                        foreach (var item1 in convex_plane){
                            d = dist_xz_points(item,item1);
                            if (d < dmin) {
                                dmin = d;
                                item.y = item1.y;
                            }
                        }
                    }
                    
                }
                update_render_cpd_result = true;
                last_time = Time.time - step_time;
                print("finish."+last_time+"rigid CPD times: "+max_step +"non-rigid times: " + max_step_non);
                if (bench_state == 4) bench_state = 2;
                if (bench_state == 5) bench_state = 3;
            
        }

        if (is_plane_register3){
            is_plane_register3 = false;
            cpd_valid_result = false;
            RVector7 sp = new RVector7();
            sp.x = nmpc_marker.transform.position.x;
            sp.y = nmpc_marker.transform.position.y;
            sp.z = nmpc_marker.transform.position.z;
            convert2nmpc(skill_point_list2,sp);
            bool docpd = false;
            nmpc_marker_object_list = new List<GameObject>();
            //foreach (var item in nmpc_marker_list){
                //GameObject g = Instantiate(plane_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
                //nmpc_marker_object_list.Add(g);
           // }
            robot_point_list.Clear();
            int i = 0;
            foreach (var item in nmpc_marker_list){
                
                if (i>10){
                    if (nmpc_marker_list[i].x < link_limit  || nmpc_marker_list[i].x > right_limit || nmpc_marker_list[i].z < down_limit|| nmpc_marker_list[i].z > up_limit){
                    robot_point_list.Add(item);
                    robot_point_list.Add(item);
                    robot_point_list.Add(item);
                    robot_point_list.Add(item);
                    robot_point_list.Add(item);
                    docpd = true;
                    break;
                }
                else {
                    robot_point_list.Add(item);
                }
                }
                else robot_point_list.Add(item);
                i++;
                
            }
            if (docpd) {
                doSkillCPD();
                
                iter++;
            }
            else {
                
                last_time = Time.time - step_time;
                print("finish. "+ last_time + "CPD: "+max_step);
            }

        }
        if (process_mode == 200 && cpd_valid_result){
            cpd_valid_result = false;
            print("point-based plane register finish.");
            skill_point_list2 = cpd_point_list;
            is_plane_register3 = true;
        }

        if (is_plane_register2){
            if (skill_point_list2.Count==0){
                foreach (var item in skill_point_list){
                    RVector7 i = new RVector7();
                    i.x = item.x;
                    i.y = item.z;
                    i.z = item.y;
                    skill_point_list2.Add(i);
                }
                

            }
            is_plane_register2 = false;
            cpd_valid_result = false;
            Vector3 whiteboard_size = new Vector3();
            whiteboard_size = find_lwh(point_list,1,1);
            Vector3 skill_size = new Vector3();
                               
            int end = 0;
            skill_size = find_lwh(skill_point_list2,1,1);
            List<RVector7> point_list3 = new List<RVector7>();

            if (whiteboard_size.x/skill_size.x < whiteboard_size.z/skill_size.z){//x-based
                float len_x = whiteboard_size.x;
                float len_z = skill_size.z * (whiteboard_size.x/skill_size.x);
                RVector7 p1 = new RVector7();
                RVector7 p2 = new RVector7();
                RVector7 p3 = new RVector7();
                RVector7 p4 = new RVector7();
                p1.x = 0;
                p1.y = 0;
                p1.z = 0;
                p2.x = 0;
                p2.y = 0;
                p2.z = len_z;
                p3.x = len_x;
                p3.y = 0;
                p3.z = len_z;
                p4.x = len_x;
                p4.y = 0;
                p4.z = 0;
                
                point_list3.Add(p1);
                point_list3.Add(p2);
                point_list3.Add(p3);
                point_list3.Add(p4);
                

            }
            else {
                float len_z = whiteboard_size.z;
                float len_x = skill_size.x * (whiteboard_size.z/skill_size.z);
                RVector7 p1 = new RVector7();
                RVector7 p2 = new RVector7();
                RVector7 p3 = new RVector7();
                RVector7 p4 = new RVector7();
                p1.x = 0;
                p1.y = 0;
                p1.z = 0;
                p2.x = 0;
                p2.y = 0;
                p2.z = len_z;
                p3.x = len_x;
                p3.y = 0;
                p3.z = len_z;
                p4.x = len_x;
                p4.y = 0;
                p4.z = 0;
                point_list3.Add(p1);
                point_list3.Add(p2);
                point_list3.Add(p3);
                point_list3.Add(p4);
                
            }
            new_skill = new List<RVector7>();
            new_skill = find_frame_points(skill_point_list2,20,0);
            reproduce_list = new List<RVector7>();
            reproduce_list = find_frame_points(point_list3,20,0);
            user_point_object_list = new List<GameObject>();
            //foreach (var item in reproduce_list)
            //{
              //  GameObject g = Instantiate(plane_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
               // user_point_object_list.Add(g);
            //}
            foreach (var item in skill_point_list2) new_skill.Add(item);
            doSkillCPD();
            


        }
        if (process_mode == 101 && cpd_valid_result){
            print("non-rigid part.");
            cpd_valid_result = false;
            user_point_object_list = new List<GameObject>();
            foreach (var item in cpd_point_list)
            {
                GameObject g = Instantiate(skillpoint_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
                user_point_object_list.Add(g);
            }
                    Vector3 mp = new Vector3();
                    mp = find_midpoint(cpd_point_list);
                    Vector3 mp4 = new Vector3();
                    mp4 = find_midpoint(point_list);
                    //float dx = whiteboard.transform.position.x - mp.x;
                    //float dz = whiteboard.transform.position.z - mp.z;
                    float dx = mp4.x - mp.x;
                    float dz = mp4.z - mp.z;
                    foreach (var item in cpd_point_list){
                        item.x += dx;
                        item.y = mp4.y;
                        item.z += dz;
                    } 

                    //update_render_cpd_result = true;
                    //check if the points out of field
                    //RVector7 np = new RVector7();
                    //List<RVector7> point_oof = new List<RVector7>();
                    if (is_register_3D){
                        new_skill = add_points(point_list,20);
                        reproduce_list = convex_plane;
                    }
                    else {
                        new_skill = find_frame_points(point_list,Mathf.FloorToInt(skill_point_list.Count/nonrigid_parameter),0);
                        reproduce_list = find_frame_points(point_list,Mathf.FloorToInt(skill_point_list.Count/nonrigid_parameter) ,1);
                        //new_skill = add_points(point_list,20);
                        //reproduce_list = add_points(point_list2,20);
                    }
                    
                    print(new_skill.Count);
                    int j = 0;
                    foreach (var item1 in cpd_point_list){
                        j++;
                        if (j >= 81){
                            new_skill.Add(item1);
                            nmpc_marker_list.Add(item1);
                        }
                       
                    }
                      print(new_skill.Count); 
                        
                    
                    //print(new_skill.Count());
                    is_rigid = false;
                    process_mode = 20;
                    doSkillCPD();
                    
                    is_plane_register = false;
                    finish_plane_register = true;

                    //entire registration p-r-p
                    /*Vector3 mpcpd = new Vector3();
            Vector3 mp = new Vector3();
            mpcpd = find_midpoint(cpd_point_list);
            mp = find_midpoint(point_list);
            Vector3 error = new Vector3();
            error.x = mpcpd.x - mp.x;
            error.y = mpcpd.y - mp.y;
            error.z = mpcpd.z - mp.z;
            nmpc_marker_list.Clear();
            int jj = 0;
            foreach (var item in cpd_point_list){
                item.x -= error.x;
                item.y -= error.y;
                item.z -= error.z;
                if (jj >= 80)
                nmpc_marker_list.Add(item);
                jj++;
            }
            
            nmpc_marker_object_list = new List<GameObject>();
            foreach (var item in nmpc_marker_list)
            {
                GameObject g = Instantiate(cpdpoint_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
                nmpc_marker_object_list.Add(g);
            }
            process_mode = 1; */   

                

        }
        if (process_mode == 100 && cpd_valid_result){
            print("Moving skill.");
            cpd_valid_result = false;
            Vector3 mpcpd = new Vector3();
            Vector3 mp = new Vector3();
            mpcpd = find_midpoint(cpd_point_list);
            mp = find_midpoint(point_list);
            Vector3 error = new Vector3();
            error.x = mpcpd.x - mp.x;
            error.y = mpcpd.y - mp.y;
            error.z = mpcpd.z - mp.z;
            nmpc_marker_list.Clear();
            foreach (var item in cpd_point_list){
                item.x -= error.x;
                item.y -= error.y;
                item.z -= error.z;
                nmpc_marker_list.Add(item);
            }
            nmpc_marker_object_list = new List<GameObject>();
            //foreach (var item in nmpc_marker_list)
            //{
                //GameObject g = Instantiate(plane_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
                //nmpc_marker_object_list.Add(g);
           // }
            List<RVector7> aa = new List<RVector7>();
            aa = find_frame_points(point_list,20,0);
            skill_point_object_list = new List<GameObject>();
            foreach (var item in aa)
            {
                GameObject g = Instantiate(skillpoint_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
                skill_point_object_list.Add(g);
            }
            last_time = Time.time - step_time;
            print("rigid finish."+last_time+"rigid CPD times: "+max_step +"non-rigid times: " + max_step_non);

        }

        if (process_mode == 2 && !is_plane_register && cpd_valid_result && is_rigid) is_plane_register = true;
        if (is_plane_register){
            print("Plane registration");
            is_plane_register = false;
            cpd_valid_result = false;

             if (skill_point_list2.Count==0){
                foreach (var item in skill_point_list){
                    RVector7 i = new RVector7();
                    i.x = item.x;
                    i.y = item.z;
                    i.z = item.y;
                    skill_point_list2.Add(i);
                }
                

            }
            

            //Vector2 whiteboard_size = new Vector2();
            //whiteboard_size.x = 10*whiteboard.transform.localScale.x;
            //whiteboard_size.y = 10*whiteboard.transform.localScale.z;
            //franka_eef_list.Clear();
            
            /*point_list = new List<RVector7>();
            RVector7 tp1 = new RVector7();
            tp1.x = point1.transform.position.x;
            tp1.y = point1.transform.position.y;
            tp1.z = point1.transform.position.z;
            RVector7 tp2 = new RVector7();
            tp2.x = point2.transform.position.x;
            tp2.y = point2.transform.position.y;
            tp2.z = point2.transform.position.z;
            RVector7 tp3 = new RVector7();
            tp3.x = point3.transform.position.x;
            tp3.y = point3.transform.position.y;
            tp3.z = point3.transform.position.z;
            RVector7 tp4 = new RVector7();
            tp4.x = point4.transform.position.x;
            tp4.y = point4.transform.position.y;
            tp4.z = point4.transform.position.z;
            point_list.Add(tp1);
            point_list.Add(tp2);
            point_list.Add(tp3);
            point_list.Add(tp4);*/
            
            
            Vector3 whiteboard_size = new Vector3();
            whiteboard_size = find_lwh(point_list,1,1);
            //print(whiteboard_size.x);
            //print(whiteboard_size.z);
           
            
            // point method
            /*if (skill_point_list.Count > 0){
                if (cpd_point_list.Count>0){
                    RVector7 s = new RVector7();
                    s.x =  start_point.x;
                    s.y =  start_point.y;
                    s.z =  start_point.z;
                    convert2nmpc(cpd_point_list,s);
                    //nmpc_marker_list = cpd_point_list;
                    
                    robot_point_list = cpd_point_list;

                }
                else{
                    RVector7 s = new RVector7();
                    s.x = start_point.x;
                    s.y = start_point.y;
                    s.z = start_point.z;
                    convert2nmpc(skill_point_list2,s);
                    robot_point_list = skill_point_list2;
                }*/ 
                RVector7 zero = new RVector7();
                zero.x = point.transform.position.x;
                zero.y = point.transform.position.y;
                zero.z = point.transform.position.z;
                if (skill_point_list.Count > 0){
                    if (cpd_point_list.Count>0){
                    
                    convert2nmpc(cpd_point_list,zero);
                    //nmpc_marker_list = cpd_point_list;
                    
                    robot_point_list = cpd_point_list;

                    }
                    else{
                    
                    convert2nmpc(skill_point_list2,zero);
                    robot_point_list = skill_point_list2;
                    }
                Vector3 skill_size = new Vector3();
                int end = 0;
                skill_size = find_lwh(nmpc_marker_list,1,1);//field method
                //print(skill_size.x/whiteboard_size.x);
                //print(skill_size.z/whiteboard_size.z);
                if (cpd_point_list.Count == 0){
                    if (skill_size.x/whiteboard_size.x > 1 || skill_size.z/whiteboard_size.z > 1)large_or_small = true;//skill larger
                }
                if (large_or_small){
                    if (skill_size.x/whiteboard_size.x > 1.2f || skill_size.z/whiteboard_size.z > 1.2f){
                        print("Larger than the field");
                        re_traj = reproduce_traj(nmpc_marker_list,0.9f);
                        /*int j = 0;
                        re_traj.Clear();
                        foreach (var item in nmpc_marker_list){
                            if (j < nmpc_marker_list.Count /2)
                            re_traj.Add(item);
                            if (j==nmpc_marker_list.Count/2){
                                re_traj.Add(item);
                                re_traj.Add(item);
                            }
                            
                            j++;

                        }*/
                        is_plane_cpd = true;
                        doSkillCPD();
                        
                    }
                    else{
                    print("Start adjust");
                    re_traj = reproduce_traj(nmpc_marker_list,0.99f);
                    is_plane_cpd = true;
                    
                    if (skill_size.x/whiteboard_size.x <= 1 && skill_size.z/whiteboard_size.z <= 1){
                        end = 1;
                        print("reproduce finish");
                    }
                    else {
                        doSkillCPD();
                        
                        }

                }
                }
                else {
                    if (skill_size.x/whiteboard_size.x < 0.8f || skill_size.z/whiteboard_size.z < 0.8f){
                    print("Smaller than the field");
                    re_traj = reproduce_traj(nmpc_marker_list,1.1f);
                    is_plane_cpd = true;
                    doSkillCPD();
                    
                }
                
                else{
                    print("Start adjust");
                    re_traj = reproduce_traj(nmpc_marker_list,1.001f);
                    is_plane_cpd = true;
                    
                    if (skill_size.x/whiteboard_size.x > 0.95f && skill_size.z/whiteboard_size.z > 0.95f){
                        end = 1;
                        print("reproduce finish");
                    }
                    else {
                        doSkillCPD();
                        
                        }
                }
                //update_render_cpd_result = true;
                }
                /*new_skill = new List<RVector7>();
                reproduce_list = new List<RVector7>();
                new_skill = find_skill_frame(skill_point_list2,skill_point_list.Count/nonrigid_parameter);
                reproduce_list = find_frame_points(point_list,skill_point_list.Count/nonrigid_parameter,1);
                foreach (var item1 in skill_point_list2){
                     new_skill.Add(item1);                        
                }
                process_mode = 29;
                doSkillCPD();*/
                if (end == 2){
                    
                    process_mode = 100;
                    cpd_valid_result = true;
                }
                

                //move the skill onto the plane                
                if (end == 1){ 
                    Vector3 mp = new Vector3();
                    mp = find_midpoint(nmpc_marker_list);
                    Vector3 mp4 = new Vector3();
                    mp4 = find_midpoint(point_list);
                    //float dx = whiteboard.transform.position.x - mp.x;
                    //float dz = whiteboard.transform.position.z - mp.z;
                    float dx = mp4.x - mp.x;
                    float dz = mp4.z - mp.z;
                    foreach (var item in nmpc_marker_list){
                        item.x += dx;
                        item.y = mp4.y;
                        item.z += dz;
                    } 

                    //update_render_cpd_result = true;
                    //check if the points out of field
                    //RVector7 np = new RVector7();
                    //List<RVector7> point_oof = new List<RVector7>();
                    if (is_register_3D){
                        new_skill = add_points(point_list,20);
                        reproduce_list = convex_plane;
                    }
                    else {
                        new_skill = find_frame_points(point_list,Mathf.FloorToInt(skill_point_list.Count/nonrigid_parameter),0);
                        reproduce_list = find_frame_points(point_list,Mathf.FloorToInt(skill_point_list.Count/nonrigid_parameter) ,1);
                        //new_skill = add_points(point_list,20);
                        //reproduce_list = add_points(point_list2,20);
                    }
                    
                    
                    
                    foreach (var item1 in nmpc_marker_list)
                        new_skill.Add(item1);
                        
                    
                    //print(new_skill.Count());
                    is_rigid = false;
                    process_mode = 20;
                    doSkillCPD();
                    
                    is_plane_register = false;
                    finish_plane_register = true;
                    

                }

              /* user_point_object_list = new List<GameObject>();
                   //nmpc_marker_object_list = new List<GameObject>();

            foreach (var item in nmpc_marker_list)
            {
                GameObject g = Instantiate(cpdpoint_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
                user_point_object_list.Add(g);
            }*/
                }
                else{
                print("Waiting for Skill.");
            }
            
            
        }

        if (process_mode == 800) {
            float d = 100;
            if (iter%100 == 0){
                eef_location.x = franka_ref.root_transform.position.x + eef_offset * franka_ref.eef_pose[1]*franka_ref.root_transform.localScale[0];
             eef_location.y = franka_ref.root_transform.position.y + franka_ref.eef_pose[2]*franka_ref.root_transform.localScale[0];
             eef_location.z = franka_ref.root_transform.position.z + franka_ref.eef_pose[0]*franka_ref.root_transform.localScale[0];
             d = dist_xz(eef_location, nmpc_marker.transform.position);
            }
            
             if (d < 0.2f) {
                 process_mode = 8;
                 RVector7 current = new RVector7();
                 RVector7 currentf = new RVector7();
                current.x = nmpc_marker.transform.position.x;
                 current.y = nmpc_marker.transform.position.y;
                current.z = nmpc_marker.transform.position.z;
                currentf.x = eef_location.x;
                currentf.y = eef_location.y;
                currentf.z = eef_location.z;
                user_point_list.Add(current);
                franka_eef_list.Add(currentf);
                print("Run index: "+user_point_list.Count);
                if (user_point_list.Count == nmpc_marker_list.Count){
                    process_mode = 0;
                    
                    last_time = Time.time - step_time;
                    print("finish."+last_time+"rigid CPD times: "+max_step +"non-rigid times: " + max_step_non);
                    RVector7 sp = new RVector7();
                sp.x = initial_nmpc_marker.x;
                sp.y = initial_nmpc_marker.y;
                sp.z = initial_nmpc_marker.z;
                convert2nmpc(skill_point_list2,sp);
                user_point_object_list = new List<GameObject>();

                foreach (var item in nmpc_marker_list)
                {
                GameObject g = Instantiate(skillpoint_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
                user_point_object_list.Add(g);
                }
                robot_point_object_list = new List<GameObject>();
                foreach (var item in franka_eef_list)
                {
                GameObject g = Instantiate(plane_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
                robot_point_object_list.Add(g);
                }

                }
             }
             else iter++;
             if (iter > 1500) {
                 print("reach joint limit");
                 iter = 0;
                
                 if (!cpd_valid_result){ 
                     robot_point_list.Clear();
                     robot_point_list = user_point_list;
                     //if (robot_point_list.Count < 10){
                         
                         //for (int ii = robot_point_list.Count;ii<11;ii++){
                             
                             //robot_point_list.Add(user_point_list[user_point_list.Count - 1]);
                             
                         //}
                         
                     //}
                     
                    if (user_point_list.Count < 11){
                        for (int j = 0;j<10;j++)
                        robot_point_list.Add(nmpc_marker_list[j]);
                    }
                    else {
                        foreach (var item in user_point_list)
                         
                         robot_point_list.Add(item);
                     
                     }
                     
                     RVector7 current = new RVector7();
                         current.x = nmpc_marker.transform.position.x;
                         current.y = nmpc_marker.transform.position.y;
                         current.z = nmpc_marker.transform.position.z;
                         robot_point_list.Add(current);
                         robot_point_list.Add(current);
                         robot_point_list.Add(current);
                         robot_point_list.Add(current);
                         robot_point_list.Add(current);
                     //process_mode == 200;
                     doSkillCPD();
                     
                 }
  
             } 
             if (cpd_valid_result){
                 
                reset();
                RVector7 sp = new RVector7();
                sp.x = nmpc_marker.transform.position.x;
                sp.y = nmpc_marker.transform.position.y;
                sp.z = nmpc_marker.transform.position.z;
                convert2nmpc(cpd_point_list,sp);
                 int ii = user_point_list.Count;
                 user_point_list.Clear();
                 int jj = 0;
                 foreach (var item in nmpc_marker_list){
                     if (jj < ii)
                     user_point_list.Add(item);
                     jj++;
                 }
                 marker_initial = true;
                 nmpc_index = nmpc_execute_index;
                 process_mode = 8;
                 
             }
        }

        if (process_mode == 8 ){
            process_mode = 800;
            cpd_valid_result = false;
             is_nmpc = false;
             iter = 0;
            //print(nmpc_index);
            if (marker_initial){
                marker_initial = false;
                nmpc_index = nmpc_execute_index;
                /*RVector7 state = new RVector7();
                state.x = initial_nmpc_marker.x;
                state.y = initial_nmpc_marker.y;
                state.z = initial_nmpc_marker.z; 
                if (nmpc_marker_list.Count == 0) convert2nmpc(skill_point_list2,state);*/
                nmpc_marker.transform.position = new Vector3(nmpc_marker_list[nmpc_execute_index].x, nmpc_marker_list[nmpc_execute_index].y, nmpc_marker_list[nmpc_execute_index].z); 
                nmpc_index++;
            }
            else{
                nmpc_marker.transform.position = new Vector3(nmpc_marker_list[nmpc_index].x, nmpc_marker_list[nmpc_index].y, nmpc_marker_list[nmpc_index].z);
                nmpc_index++;
            }
            nmpc_execute_index = nmpc_index - 1;
            
        
        }

        
        //
         //if (!is_nmpc && (process_mode == 1 || process_mode == 9 || process_mode == 81)){
          if (process_mode == 11){
              float d = 100;
                 int t = 350;
                 if (iter%50 == 0){
                    eef_location.x = franka_ref.root_transform.position.x + eef_offset * franka_ref.eef_pose[1]*franka_ref.root_transform.localScale[0];
                    eef_location.y = franka_ref.root_transform.position.y + franka_ref.eef_pose[2]*franka_ref.root_transform.localScale[0];
                    eef_location.z = franka_ref.root_transform.position.z + franka_ref.eef_pose[0]*franka_ref.root_transform.localScale[0];
                    d = dist_xz(eef_location, nmpc_marker.transform.position);
                    print("Current distance: "+d);
                    //iter++;
                 }
                if (close_far == 99) t = 3000;
             if (d < 0.2f) {
                 process_mode = 9;
                 if (close_far != 99){
                     if (close_or_far == 0)
                         close_or_far = 1;
                     else
                         close_or_far = 0;
                 }
                 
                 close_far = 99;
                 iter = 0;
                 close_and_far++;
                //robot_point_list.Add(current);
                print(close_and_far);
                if (close_and_far >= 2){
                    process_mode = 0;
                    last_time = Time.time - step_time;
                    print("Finish in "+last_time+"s, rigid CPD times: "+max_step +", non-rigid CPD times: " + max_step_non);
                    print("Skill scale: "+ cpdscale);
                }
             }
             else iter++;
             if (iter > t) {
                 
                  print("reach joint limit");
                 iter = 0;
                 close_and_far = 0;
                 if (close_far == 99){
                    if (close_or_far == 1){
                     close_or_far = 0;
                     close_far = 0;
                 }
                 
                 else{
                     close_or_far = 1;
                     close_far = 1;
                 }
                  
                 }
                 else 
                 {
                     if (close_or_far == 0){
                     close_or_far = 0;
                     close_far = 0;
                 }
                 
                 else{
                     close_or_far = 1;
                     close_far = 1;
                 }
                 }
                 Vector3 mp = new Vector3();
                 mp = find_midpoint(point_list);
                 
                 if (Mathf.Abs(eef_location.x - nmpc_marker.transform.position.x) > Mathf.Abs(eef_location.z - nmpc_marker.transform.position.z)){//boundary 1,3
                     if (nmpc_marker.transform.position.x - mp.x < 0) boundary_mp = find_frame_midpoint(point_list,3);
                     else boundary_mp = find_frame_midpoint(point_list,1);
                 }
                 else {
                    if (nmpc_marker.transform.position.z - mp.z < 0) boundary_mp = find_frame_midpoint(point_list,2);
                     else boundary_mp = find_frame_midpoint(point_list,4); 
                 }
                 if (!cpd_valid_result){
                     nmpc_marker_list.Add(boundary_mp);
                     re_traj = reproduce_traj(nmpc_marker_list,0.9f);
                     is_plane_cpd = true;
                     doSkillCPD();
                     
                 }
  
             } 
             if (cpd_valid_result){
                 //cpd_valid_result = false;
                 float dx = boundary_mp.x - cpd_point_list[cpd_point_list.Count - 1].x;
                 float dz = boundary_mp.z - cpd_point_list[cpd_point_list.Count - 1].z;
                 
                 foreach (var item in cpd_point_list){
                     item.x += dx;
                     item.y = boundary_mp.y;
                     item.z += dz;
                     
                 }
                 cpd_point_list.Remove(cpd_point_list[cpd_point_list.Count - 1]);
                 nmpc_marker_list = cpd_point_list;
                 print(nmpc_marker_list.Count);
                 process_mode = 1;
                 nmpc_index = 0;
                 cpd_valid_result = false;
                 marker_initial = true;
                 //update_render_cpd_result = true;
                 reset();
                 
                 //update_render_cpd_result = false;

             
            
             }
              
         }
        //T: record the skill
         if (process_mode == 71){
                 float d = 100;
                 if (iter%50 == 0){
                    eef_location.x = franka_ref.root_transform.position.x + eef_offset * franka_ref.eef_pose[1]*franka_ref.root_transform.localScale[0];
                    eef_location.y = franka_ref.root_transform.position.y + franka_ref.eef_pose[2]*franka_ref.root_transform.localScale[0];
                    eef_location.z = franka_ref.root_transform.position.z + franka_ref.eef_pose[0]*franka_ref.root_transform.localScale[0];
                    d = dist_xz(eef_location, nmpc_marker.transform.position);
                    //d = dist_xy(eef_location, nmpc_marker.transform.position);
                    //print("Current distance: "+eef_location.x+"||"+eef_location.z+"||"+nmpc_marker.transform.position.x+"||"+nmpc_marker.transform.position.z);
                    //iter++;
                 }
                
             if (d < 0.2f) {
                 process_mode = 7;
                 RVector7 current = new RVector7();
                 current.x = eef_location.x;
                 current.y = eef_location.y;
                 current.z = eef_location.z;
                 robot_point_list.Add(current);
                 print("Current step: "+(nmpc_index+1));
                 robot_point_object_list = new List<GameObject>();
                foreach (var item in robot_point_list)
                {
                GameObject g = Instantiate(plane_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
                robot_point_object_list.Add(g);
                }
                //robot_point_list.Add(current);
                 string ss=franka_ref.arm_1 + "," + franka_ref.arm_2 + "," + franka_ref.arm_3 + "," + franka_ref.arm_4 + "," + franka_ref.arm_5 + "," + franka_ref.arm_6 + "," + franka_ref.arm_7;
                 joint_position_list.Add(ss);
                
             }
             else iter++;
             
             
              
         }

         if (process_mode == 7){
             cpd_valid_result = false;
             process_mode = 71;
             iter = 0;
            //print(nmpc_index);
            if (marker_initial){
                marker_initial = false;
                robot_point_list.Clear();
                /*RVector7 state = new RVector7();
                state.x = initial_nmpc_marker.x;
                state.y = initial_nmpc_marker.y;
                state.z = initial_nmpc_marker.z; 
                if (nmpc_marker_list.Count == 0) convert2nmpc(skill_point_list2,state);*/
                nmpc_marker.transform.position = new Vector3(nmpc_marker_list[0].x, nmpc_marker_list[0].y, nmpc_marker_list[0].z); 
            foreach (var item in cpd_point_object_list)
            {
                Destroy(item);
            }
            foreach (var item in robot_point_object_list)
            {
                Destroy(item);
            }
            foreach (var item in skill_point_object_list)
            {
                Destroy(item);
            }
            foreach (var item in user_point_object_list)
            {
                Destroy(item);
            }
            foreach (var item in nmpc_marker_object_list)
            {
                Destroy(item);
            }
            nmpc_marker_object_list = new List<GameObject>();
            foreach (var item in nmpc_marker_list)
            {
                GameObject g = Instantiate(skillpoint_prefab, Vector3.zero, Quaternion.identity);

                g.transform.localPosition = new Vector3(item.x, item.y, item.z);

                nmpc_marker_object_list.Add(g);
            }
                //nmpc_index = 1;
            }
            else{
                nmpc_index++;
                if (nmpc_index >= nmpc_marker_list.Count) {
                process_mode = 0;
                print("Record Skill.");
                
                nmpc_marker_object_list = new List<GameObject>();
                foreach (var item in nmpc_marker_list)
                {
                GameObject g = Instantiate(plane_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
                nmpc_marker_object_list.Add(g);
                }
                FileStream fs1 = new FileStream(Application.dataPath + "/Benchmarks/skills/franka_joint_state", FileMode.Create, FileAccess.Write);
                StreamWriter sw1 = new StreamWriter(fs1);

            
               foreach (var item in joint_position_list)
            {
                
                
                sw1.WriteLine(item);
            }
            print("save joint state");

            sw1.Close();
            fs1.Close();

                FileStream fs = new FileStream(Application.dataPath + "/Benchmarks/skills/franka_skill", FileMode.Create, FileAccess.Write);
                StreamWriter sw = new StreamWriter(fs);

            int i = 0;
            foreach (var item in nmpc_marker_list)
            {
                i++;
                item.x -= franka_ref.root_transform.position.x;
                item.y -= franka_ref.root_transform.position.y;
                item.z -= franka_ref.root_transform.position.z;
                sw.WriteLine(item.x + "," + item.y + "," + item.z + "," + item.qx + "," + item.qy + "," + item.qz + "," + item.qw);
            }

            print("save skill with " + i.ToString());

            sw.Close();
            fs.Close();
            } 
            else nmpc_marker.transform.position = new Vector3(nmpc_marker_list[nmpc_index].x, nmpc_marker_list[nmpc_index].y, nmpc_marker_list[nmpc_index].z);
                
            }
               
            }

            if (process_mode == 9 || process_mode ==1){
                if (process_mode == 9) process_mode = 900;
                else process_mode = 11;

                print("Start checking the limit points.");
                //iter = 0;
                //is_nmpc = false;
                float min_d = 9999;
                float max_d = -9999;
                RVector7 franka_point = new RVector7();
                franka_point.x = franka_ref.transform.position.x;
                franka_point.y = franka_ref.transform.position.y;
                franka_point.z = franka_ref.transform.position.z;
                RVector7 min_nmpc = new RVector7();
                RVector7 max_nmpc = new RVector7();
                foreach (var item in nmpc_marker_list){
                    if (dist(franka_point,item) < min_d){
                        min_nmpc.x = item.x;
                        min_nmpc.y = item.y;
                        min_nmpc.z = item.z;
                        min_d = dist(franka_point,item);
                    }
                    if (dist(franka_point,item) > max_d){
                        max_nmpc.x = item.x;
                        max_nmpc.y = item.y;
                        max_nmpc.z = item.z;
                        max_d = dist(franka_point,item);
                    }
                }
                
                
                if (close_or_far == 0){
                    nmpc_marker.transform.position = new Vector3(max_nmpc.x,max_nmpc.y,max_nmpc.z);
                    if (close_far == 0)
                    close_or_far = 0;
                    else
                    close_or_far = 1;
                    //close_and_far++;
                }
                else if (close_or_far == 1) {
                    nmpc_marker.transform.position = new Vector3(min_nmpc.x,min_nmpc.y,min_nmpc.z);
                    if (close_far == 1)
                    close_or_far = 1;
                    else
                    close_or_far = 0;
                    //close_and_far++;
                }
                
            }

             if (process_mode == 900) {
                 float d = 100;
                 int t = 300;
                 if (iter%50 == 0){
                    eef_location.x = franka_ref.root_transform.position.x + eef_offset * franka_ref.eef_pose[1]*franka_ref.root_transform.localScale[0];
                    eef_location.y = franka_ref.root_transform.position.y + franka_ref.eef_pose[2]*franka_ref.root_transform.localScale[0];
                    eef_location.z = franka_ref.root_transform.position.z + franka_ref.eef_pose[0]*franka_ref.root_transform.localScale[0];
                    d = dist_xz(eef_location, nmpc_marker.transform.position);
                    print("Current distance: "+d);
                    //iter++;
                 }
                if (close_far == 99) t = 2000;
             if (d < 0.2f) {
                 process_mode = 9;
                 if (close_far != 99){
                     if (close_or_far == 0)
                         close_or_far = 1;
                     else
                         close_or_far = 0;
                 }
                 
                 close_far = 99;
                 iter = 0;
                 close_and_far++;
                //robot_point_list.Add(current);
                print(close_and_far);
                if (close_and_far >= 2){
                    process_mode = 0;
                    last_time = Time.time - step_time;
                    print("Finish in "+last_time+"s, rigid CPD times: "+max_step +", non-rigid CPD times: " + max_step_non);
                    print("Skill scale: "+ cpdscale);
                    
                }
             }
             else iter++;
             if (iter > t) {
                 print("reach joint limit");
                 iter = 0;
                 close_and_far = 0;
                 if (close_far == 99){
                    if (close_or_far == 1){
                     close_or_far = 0;
                     close_far = 0;
                 }
                 
                 else{
                     close_or_far = 1;
                     close_far = 1;
                 }
                  
                 }
                 else 
                 {
                     if (close_or_far == 0){
                     close_or_far = 0;
                     close_far = 0;
                 }
                 
                 else{
                     close_or_far = 1;
                     close_far = 1;
                 }
                 }
                 
                
                 if (!cpd_valid_result){
                     
                         int ii = 0;
                         foreach (var item in nmpc_marker_list){
                             
                             robot_point_list.Add(item);
                             if (item.x == nmpc_marker.transform.position.x && item.z == nmpc_marker.transform.position.z){
                                 robot_point_list.Add(item);
                                 robot_point_list.Add(item);
                                 robot_point_list.Add(item);
                                 robot_point_list.Add(item);
                                 robot_point_list.Add(item);
                                 break;
                             }
                             
                         }
                         
                     
                     /*RVector7 current = new RVector7();
                         current.x = nmpc_marker.transform.position.x;
                         current.y = nmpc_marker.transform.position.y;
                         current.z = nmpc_marker.transform.position.z;
                         robot_point_list.Add(current);
                         robot_point_list.Add(current);
                         robot_point_list.Add(current);
                         robot_point_list.Add(current);
                         robot_point_list.Add(current);*/
                     //process_mode == 200;
                     doSkillCPD();
                     
                 }
  
             }
             if (cpd_valid_result){
                 reset();
                // cpd_valid_result = false;
                RVector7 sp = new RVector7();
                sp.x = nmpc_marker.transform.position.x;
                sp.y = nmpc_marker.transform.position.y;
                sp.z = nmpc_marker.transform.position.z;
                convert2nmpc(cpd_point_list,sp);
               
            
                 process_mode = 9;
                 
                 marker_initial = true;
                 cpd_valid_result = false;
                 nmpc_index = 0;
                 
             }
        
             }           
            /*if (is_reproduce_franka){
                bool resize_skill = false;
                
               
                if (resize_skill){ 
                    resize_skill = false;
                    if (nmpc_marker_list.Count != 0){
                        re_traj = reproduce_traj(nmpc_marker_list,0.99f);
                        is_plane_cpd = true;
                        doSkillCPD();
                    }
                    reset();
                }
            }*/
        
            
        //print(franka_ref.arm_7);
        
    }

    float cdist(RVector7 a, Vector3 b)
    {
        return Mathf.Sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
    }

    void saveCurrentBenchStep(string mode, string method, string name, bool total = false)
    {
        print("Save the current benchmark");

        FileStream fs = new FileStream(Application.dataPath + "/Benchmarks/" + mode + "_" + method + ".txt", FileMode.Append, FileAccess.Write);
        StreamWriter sw = new StreamWriter(fs);

        if (total == false)
            sw.WriteLine(name + " " + p_max_distance + " " + p_tracking_error + " " + p_robot_traj_len + " " + p_user_traj_len);
        else
        {
            sw.WriteLine(name + " " + p_max_distance + " " + p_tracking_error + " " + p_robot_traj_len + " " + p_user_traj_len);
            sw.WriteLine(name + " " + total_max_distance + " " + total_tracking_error + " " + total_robot_traj_len + " " + total_user_traj_len);
        }

        sw.Close();
        fs.Close();
    }

    public void Update()
    {
        if (reset_trail > 0)
        {
            user.GetComponent<TrailRenderer>().time = 0;
            robot.GetComponent<TrailRenderer>().time = 0;

            reset_trail++;

            if (reset_trail > 30)
            {
                user.GetComponent<TrailRenderer>().time = 100;
                robot.GetComponent<TrailRenderer>().time = 100;
                reset_trail = 0;
            }
        }

        if (Input.GetKeyDown(KeyCode.E))
        {
            print("Start / Stop scenario recording");

            if (start_recording_scenario == false)
            {
                scenario_point_list = new List<RVector7>();
            }
            start_recording_scenario = !start_recording_scenario;
        }

        if (Input.GetKeyDown(KeyCode.P))
        {
            KeyP();
        }

        if (Input.GetKeyDown(KeyCode.X))
        {
            KeyX();
        }

        if (Input.GetKeyDown(KeyCode.R))
        {
            print("Start / Stop recording skill");

            if (start_recording_skill == false)
            {
                skill_point_list = new List<RVector7>();
            }
            start_recording_skill = !start_recording_skill;
        }

        if (Input.GetKeyDown(KeyCode.S))
        {
            print("Save the recorded skill");

            FileStream fs = new FileStream(Application.dataPath + "/Benchmarks/skills/" + skill_file_name, FileMode.Create, FileAccess.Write);
            StreamWriter sw = new StreamWriter(fs);

            int i = 0;
            foreach (var item in skill_point_list)
            {
                i++;
                sw.WriteLine(item.x + "," + item.y + "," + item.z + "," + item.qx + "," + item.qy + "," + item.qz + "," + item.qw);
            }

            print("save skill with " + i.ToString());

            sw.Close();
            fs.Close();
        }

        if (Input.GetKeyDown(KeyCode.W))
        {
            print("Save the scenario");

            FileStream fs = new FileStream(Application.dataPath + "/Benchmarks/scenarios/" + scenario_file_name, FileMode.Create, FileAccess.Write);
            StreamWriter sw = new StreamWriter(fs);

            int i = 0;
            foreach (var item in scenario_point_list)
            {
                i++;
                sw.WriteLine(item.x + "," + item.y + "," + item.z + "," + item.qx + "," + item.qy + "," + item.qz + "," + item.qw);
            }

            print("Save the scenario with " + i.ToString());

            sw.Close();
            fs.Close();
        }

        if (Input.GetKeyDown(KeyCode.L))
        {
            KeyL();
        }

        if (Input.GetKeyDown(KeyCode.O))
        {
            KeyO();
        }

        if (Input.GetKeyDown(KeyCode.C))
        {
            KeyC();
        }

        if (Input.GetKeyDown(KeyCode.D))
        {
            KeyD();
        }
        
        if (Input.GetKeyDown(KeyCode.K))
        {
            KeyK();
        }
        if (Input.GetKeyDown(KeyCode.J))
        {
            KeyJ();
        }
        if (Input.GetKeyDown(KeyCode.H))
        {
            KeyH();
        }
        if (Input.GetKeyDown(KeyCode.N))
        {
            KeyN();
        }

        if (Input.GetKeyDown(KeyCode.T))
        {
            KeyT();
        }

        if (Input.GetKeyDown(KeyCode.B))
        {
            KeyB();
        }

        if (Input.GetKeyDown(KeyCode.E))
        {
            KeyE();
        }

        if (Input.GetKeyDown(KeyCode.I))
        {
            KeyI();
        }

        if (Input.GetKeyDown(KeyCode.M))
        {
            KeyM();
        }
        

        if (update_render_cpd_result)
        {
            update_render_cpd_result = false;

            foreach (var item in cpd_point_object_list)
            {
                Destroy(item);
            }

            foreach (var item in nmpc_marker_object_list)
            {
                Destroy(item);
            }

            cpd_point_object_list = new List<GameObject>();
            nmpc_marker_object_list = new List<GameObject>();

            foreach (var item in cpd_point_list)
            {
                GameObject g = Instantiate(cpdpoint_prefab, Vector3.zero, Quaternion.identity);

                g.transform.localPosition = new Vector3(item.x, item.y, item.z);

                cpd_point_object_list.Add(g);
            }

            /*foreach (var item in nmpc_marker_list)
            {
                GameObject g = Instantiate(nmpc_prefab, Vector3.zero, Quaternion.identity);

                g.transform.localPosition = new Vector3(item.x, item.y, item.z);

                nmpc_marker_object_list.Add(g);
            }*/
        }

        if (bench_state == 1)
        {
            
            print("Benchmark started.");

            skill_file_name = bench_index.ToString() + ".txt";
            //scenario_file_name = bench_index.ToString() + ".txt";
            //bench_index = 5;
            //Reset Everything
            //resetScenario();
            if (bench_index < 4){
                print("Regular plane, iteration: " + bench_index);
                point_list = new List<RVector7>();
                point_list2 = new List<RVector7>();
                RVector7 tp1 = new RVector7();
                tp1.x = -48 - bench_index;
            tp1.y = 17;
            tp1.z = 10 - bench_index;
            RVector7 tp2 = new RVector7();
            tp2.x = -48 - bench_index;
            tp2.y = 17;
            tp2.z = 22 - bench_index;
            RVector7 tp3 = new RVector7();
            tp3.x = -38 - bench_index;
            tp3.y = 17;
            tp3.z = 22 - bench_index;
            RVector7 tp4 = new RVector7();
            tp4.x = -38 - bench_index;
            tp4.y = 17;
            tp4.z = 10 - bench_index;
            point_list.Add(tp1);
            point_list.Add(tp2);
            point_list.Add(tp3);
            point_list.Add(tp4);
            point_list2.Add(tp1);
            point_list2.Add(tp2);
            point_list2.Add(tp3);
            point_list2.Add(tp4);
            point1.transform.position = new Vector3(tp1.x,tp1.y,tp1.z);
            point2.transform.position = new Vector3(tp2.x,tp2.y,tp2.z);
            point3.transform.position = new Vector3(tp3.x,tp3.y,tp3.z);
            point4.transform.position = new Vector3(tp4.x,tp4.y,tp4.z);
            //Load Skill
            KeyL();

            //Load Scenario
            KeyK();
            }
            if (bench_index >=4 && bench_index < 7){
                print("Irregural Plane, iteration: " + bench_index);
                point_list = new List<RVector7>();
                point_list2 = new List<RVector7>();
                RVector7 tp1 = new RVector7();
                tp1.x = -50 - Random.Range(0,5);
            tp1.y = 17;
            tp1.z = 10 - Random.Range(0,5);
            RVector7 tp2 = new RVector7();
            tp2.x = -50 - Random.Range(0,5);
            tp2.y = 17;
            tp2.z = 22 - Random.Range(0,5);
            RVector7 tp3 = new RVector7();
            tp3.x = -38 - Random.Range(0,5);
            tp3.y = 17;
            tp3.z = 22 - Random.Range(0,5);
            RVector7 tp4 = new RVector7();
            tp4.x = -38 - Random.Range(0,5);
            tp4.y = 17;
            tp4.z = 10 - Random.Range(0,5);
            point_list.Add(tp1);
            point_list.Add(tp2);
            point_list.Add(tp3);
            point_list.Add(tp4);
            point_list2.Add(tp1);
            point_list2.Add(tp2);
            point_list2.Add(tp3);
            point_list2.Add(tp4);
            point1.transform.position = new Vector3(tp1.x,tp1.y,tp1.z);
            point2.transform.position = new Vector3(tp2.x,tp2.y,tp2.z);
            point3.transform.position = new Vector3(tp3.x,tp3.y,tp3.z);
            point4.transform.position = new Vector3(tp4.x,tp4.y,tp4.z);

            //Load Skill
            KeyL();

            //Load Scenario
            KeyK();
            }
            //Track
            //KeyB();
            if (bench_index >= 7 && bench_index < 12){
                print("3D Plane, iteration: " + bench_index);
                point_list = new List<RVector7>();
                point_list2 = new List<RVector7>();
                RVector7 tp1 = new RVector7();
                tp1.x = -50 - Random.Range(0,5);
            tp1.y = 17;
            tp1.z = 10 - Random.Range(0,5);
            RVector7 tp2 = new RVector7();
            tp2.x = -50 - Random.Range(0,5);
            tp2.y = 17;
            tp2.z = 22 - Random.Range(0,5);
            RVector7 tp3 = new RVector7();
            tp3.x = -38 - Random.Range(0,5);
            tp3.y = 17;
            tp3.z = 22 - Random.Range(0,5);
            RVector7 tp4 = new RVector7();
            tp4.x = -38 - Random.Range(0,5);
            tp4.y = 17;
            tp4.z = 10 - Random.Range(0,5);
            point_list.Add(tp1);
            point_list.Add(tp2);
            point_list.Add(tp3);
            point_list.Add(tp4);
            point_list2.Add(tp1);
            point_list2.Add(tp2);
            point_list2.Add(tp3);
            point_list2.Add(tp4);
            point1.transform.position = new Vector3(tp1.x,tp1.y,tp1.z);
            point2.transform.position = new Vector3(tp2.x,tp2.y,tp2.z);
            point3.transform.position = new Vector3(tp3.x,tp3.y,tp3.z);
            point4.transform.position = new Vector3(tp4.x,tp4.y,tp4.z);

            //Load Skill
            KeyL();
            if (Random.Range(0,3) != 0)
            KeyC();
            KeyC();
            //Load Scenario
            KeyK();
            }

            if (bench_index >= 12 && bench_index < 15){
                print("3D Plane, double nonrigid, iteration: " + bench_index);
                point_list = new List<RVector7>();
                point_list2 = new List<RVector7>();
                RVector7 tp1 = new RVector7();
                tp1.x = -50 - Random.Range(0,5);
            tp1.y = 17;
            tp1.z = 10 - Random.Range(0,5);
            RVector7 tp2 = new RVector7();
            tp2.x = -50 - Random.Range(0,5);
            tp2.y = 17;
            tp2.z = 22 - Random.Range(0,5);
            RVector7 tp3 = new RVector7();
            tp3.x = -38 - Random.Range(0,5);
            tp3.y = 17;
            tp3.z = 22 - Random.Range(0,5);
            RVector7 tp4 = new RVector7();
            tp4.x = -38 - Random.Range(0,5);
            tp4.y = 17;
            tp4.z = 10 - Random.Range(0,5);
            point_list.Add(tp1);
            point_list.Add(tp2);
            point_list.Add(tp3);
            point_list.Add(tp4);
            point_list2.Add(tp1);
            point_list2.Add(tp2);
            point_list2.Add(tp3);
            point_list2.Add(tp4);
            point1.transform.position = new Vector3(tp1.x,tp1.y,tp1.z);
            point2.transform.position = new Vector3(tp2.x,tp2.y,tp2.z);
            point3.transform.position = new Vector3(tp3.x,tp3.y,tp3.z);
            point4.transform.position = new Vector3(tp4.x,tp4.y,tp4.z);

            //Load Skill
            KeyL();
            KeyD();
            if (Random.Range(0,3) != 0)
            KeyC();
            KeyC();
            //Load Scenario
            KeyK();
            }
            //Play Scenario
            //is_play_scenario = !is_play_scenario;
            if (bench_index >= 15 && bench_index < 20){
                print("2D Plane in space, iteration: " + bench_index);
                point_list = new List<RVector7>();
                point_list2 = new List<RVector7>();
                RVector7 tp1 = new RVector7();
                tp1.x = -50 - Random.Range(0,3);
            tp1.y = 17;
            tp1.z = 10 - Random.Range(0,3);
            RVector7 tp2 = new RVector7();
            tp2.x = -64 - Random.Range(0,3);
            tp2.y = 17;
            tp2.z = 10 - Random.Range(0,3);
            RVector7 tp3 = new RVector7();
            tp3.x = -55 - Random.Range(0,3);
            tp3.y = 17;
            tp3.z = 27 - Random.Range(0,3);
            RVector7 tp4 = new RVector7();
            tp4.x = -40 - Random.Range(0,3);
            tp4.y = 17;
            tp4.z = 21 - Random.Range(0,3);
            point_list.Add(tp1);
            point_list.Add(tp2);
            point_list.Add(tp3);
            point_list.Add(tp4);
            point_list2.Add(tp1);
            point_list2.Add(tp2);
            point_list2.Add(tp3);
            point_list2.Add(tp4);
            point1.transform.position = new Vector3(tp1.x,tp1.y,tp1.z);
            point2.transform.position = new Vector3(tp2.x,tp2.y,tp2.z);
            point3.transform.position = new Vector3(tp3.x,tp3.y,tp3.z);
            point4.transform.position = new Vector3(tp4.x,tp4.y,tp4.z);
            KeyO();
            //Load Skill
            FixedUpdate();
            KeyL();
            
            if (process_mode == 7)
            KeyK();
            //Load Scenario
            //KeyO();
            }

            if (bench_index == 27){//lmt in irregular plane

            }

            if (bench_index == 28){//random 3 letters in 3D plane

            }

            if (bench_index == 29){//random 3 letters in space

            }

            if (bench_index == 30){//random 3 letters in 3D space

            }

            if (bench_index == 31){
                print("Benchmark ended.");
                bench_state = 0;
            }
            if (bench_index >= 15 && bench_index < 20) bench_state = 5;
            else
            bench_state = 4;
        }

        if (bench_state == 3) {
            KeyO();
            bench_state = 10;
        }
        if (bench_state == 2)
        {
            if (bench_index >= 15 && bench_index < 20) KeyO();
            nmpc_marker_list.Add(point_list2[0]);
            nmpc_marker_list.Add(point_list2[1]);
            nmpc_marker_list.Add(point_list2[2]);
            nmpc_marker_list.Add(point_list2[3]);
            Vector3 mp = find_midpoint(nmpc_marker_list);
            Vector3 error = new Vector3();
            error.x = mp.x - benchmark_board.transform.position.x;
            error.y = mp.y - benchmark_board.transform.position.y;
            error.z = mp.z - benchmark_board.transform.position.z;
            foreach (var item in nmpc_marker_list){
                item.x -= error.x;
                item.y -= error.y;
                item.z -= error.z;
            }
            if (bench_index == 9 || bench_index == 18)
            benchmark_board.transform.position = new Vector3(benchmark_board.transform.position.x - 160, benchmark_board.transform.position.y,benchmark_board.transform.position.z - 20);
            else
            benchmark_board.transform.position = new Vector3(benchmark_board.transform.position.x + 20, benchmark_board.transform.position.y,benchmark_board.transform.position.z);
            foreach (var item in nmpc_marker_object_list)
            {
                Destroy(item);
            }
            nmpc_marker_object_list = new List<GameObject>();

            foreach (var item in nmpc_marker_list)
            {
                GameObject g = Instantiate(nmpc_prefab, Vector3.zero, Quaternion.identity);

                g.transform.localPosition = new Vector3(item.x, item.y, item.z);

                nmpc_marker_object_list.Add(g);
            }
            resetScenario();
            
            bench_index++;
            bench_state = 1;
            /*if (scenario_index >= (scenario_point_list.Count / 2) && disconnetion_index == 0) // disconnet on 0.5
            {
                disconnetion_index = 1;
                print("Disconnet 0.5");
                //Disconnet Network
                KeyT();
            }

            if (scenario_index >= (3 * scenario_point_list.Count / 4) && disconnetion_index == 1) //bring back on 0.75
            {
                disconnetion_index = 2;
                print("Connect 0.75");
                //Connect Network
                KeyT();
            }

            if (scenario_index >= scenario_point_list.Count)
            {
                //Done
                print("Test done for " + bench_index.ToString());

                KeyP();

                total_max_distance += p_max_distance;
                total_tracking_error += p_tracking_error;
                total_user_traj_len += p_user_traj_len;
                total_robot_traj_len += p_robot_traj_len;

                if (bench_index < 26)
                {
                    total_max_distance = total_max_distance / 26;
                    total_tracking_error = total_tracking_error / 26;
                    total_user_traj_len = total_user_traj_len / 26;
                    total_robot_traj_len = total_robot_traj_len / 26;

                    print("Avg max_distance = " + total_max_distance.ToString());
                    print("Avg total_tracking_error = " + total_tracking_error.ToString());
                    print("Avg user_traj_len = " + total_user_traj_len.ToString());
                    print("Avg robot_traj_len = " + total_robot_traj_len.ToString());

                    saveCurrentBenchStep(mode, operation_mpde.ToString(), bench_index.ToString(), false);

                    bench_index++;
                    bench_state = 1;

                }
                else
                {
                    print("benchmark finish");

                    saveCurrentBenchStep(mode, operation_mpde.ToString(), bench_index.ToString(), true);
                    bench_state = 3;
                }
            }*/
        }
    }

    float calculate_max_delta_d()
    {
        float max_d = -1;
        RVector7 old = null;
        bool skip_first = false;

        foreach (var item in robot_point_list)
        {
            if (skip_first == false)
            {
                skip_first = true;
                continue;
            }

            if (old == null)
            {
                old = item;
            }
            else
            {
                float d = dist(item, old);
                if (d > max_d)
                    max_d = d;
                old = item;
            }
        }

        return max_d;
    }

    float calculate_user_tracking_error()
    {
        float e = 0;
        int x = 0;
        foreach (var item in user_point_object_list)
        {
            float d = findClosestDistinSkill(item, robot_point_object_list);

            if (d < delta_d)
            {
                //ok
            }
            else
            {
                //real error
                x++;
                e += d;
            }

        }

        if (x != 0)
            e = e / x;
        else
            e = 0;

        return e;
    }

    float calculate_user_traj_length()
    {
        float len = 0;
        RVector7 old = null;

        foreach (var item in user_point_list)
        {
            if (old == null)
            {
                old = item;
            }
            else
            {
                float d = dist(item, old);
                len += d;
                old = item;
            }
        }
        return len;
    }

    float calculate_robot_traj_length()
    {
        float len = 0;
        RVector7 old = null;

        foreach (var item in robot_point_list)
        {
            if (old == null)
            {
                old = item;
            }
            else
            {
                float d = dist(item, old);
                len += d;
                old = item;
            }
        }
        return len;
    }

    void removePoints()
    {
        foreach (var item in skill_point_object_list)
        {
            Destroy(item);
        }

        foreach (var item in cpd_point_object_list)
        {
            Destroy(item);
        }

        foreach (var item in robot_point_object_list)
        {
            Destroy(item);
        }

        foreach (var item in user_point_object_list)
        {
            Destroy(item);
        }

        skill_point_object_list = new List<GameObject>();
        cpd_point_object_list = new List<GameObject>();
        robot_point_object_list = new List<GameObject>();
        user_point_object_list = new List<GameObject>();
    }

    void doSkillCPD()
    {
        RRSCPDCommand cpd_command = new RRSCPDCommand();
        //Current User Trajectory
        
        int robot_count;
        if (!is_rigid){
            max_step_non++;
            print("irregular rechtangle");
            //is_rigid = true;
            is_plane_cpd = false;
            robot_count = reproduce_list.Count;
            cpd_command.points_skill = new_skill.ToArray();
            cpd_command.points_a = reproduce_list.ToArray();
            cpd_command.mode = 1;
            cpd_command.points_b = reproduce_list.ToArray();

        }
        else {
            
            if (process_mode == 2){
            is_plane_cpd = false;
            robot_count = re_traj.Count;
            cpd_command.points_a = re_traj.ToArray();
            cpd_command.points_skill = skill_point_list2.ToArray();
            cpd_command.mode = 0;
            max_step++;
            List<RVector7> cut_skill_list = new List<RVector7>();
            int i = 0;
            foreach (var item in skill_point_list2)
            {
                cut_skill_list.Add(item);
                i++;

                if (i >= robot_count)
                {
                    break;
                }
            }

            cpd_command.points_b = cut_skill_list.ToArray();
        }
        else if (process_mode == 29){
            is_plane_cpd = false;
            process_mode = 28;
            robot_count = reproduce_list.Count;
            cpd_command.points_skill = new_skill.ToArray();
            cpd_command.points_a = reproduce_list.ToArray();
            cpd_command.mode = 1;
            max_step_non++;
            List<RVector7> cut_skill_list = new List<RVector7>();

            int i = 0;
            foreach (var item in new_skill)
            {
                cut_skill_list.Add(item);
                i++;

                if (i >= robot_count)
                {
                    break;
                }
            }

            cpd_command.points_b = cut_skill_list.ToArray();
        }
        else if (process_mode == 3){
            is_skill_cpd = false;
            robot_count = re_traj.Count;
            cpd_command.points_a = re_traj.ToArray();
            cpd_command.points_skill = skill_point_list2.ToArray();
            cpd_command.mode = 0;
            max_step++;
            cpd_command.points_b = skill_point_list2.ToArray();
        }
        else if (process_mode == 100){
            process_mode = 101;
            robot_count = reproduce_list.Count;
            cpd_command.points_skill = new_skill.ToArray();
            cpd_command.points_a = reproduce_list.ToArray();
            cpd_command.mode = 0;
            max_step++;
            List<RVector7> cut_skill_list = new List<RVector7>();

            int i = 0;
            foreach (var item in new_skill)
            {
                cut_skill_list.Add(item);
                i++;

                if (i >= robot_count)
                {
                    break;
                }
            }

            cpd_command.points_b = cut_skill_list.ToArray();
        }
        else if (process_mode == 200 || process_mode == 800 || process_mode == 900){
            robot_count = robot_point_list.Count;
            cpd_command.points_skill = skill_point_list2.ToArray();
            cpd_command.points_a = robot_point_list.ToArray();
            cpd_command.mode = 0;
            max_step++;
            List<RVector7> cut_skill_list = new List<RVector7>();

            int i = 0;
            foreach (var item in skill_point_list2)
            {
                cut_skill_list.Add(item);
                i++;

                if (i >= robot_count)
                {
                    break;
                }
            }

            cpd_command.points_b = cut_skill_list.ToArray();
        }
        else if (process_mode == 80){
            print("Combination CPD.");
            robot_count = re_traj.Count;
            cpd_command.points_a = re_traj.ToArray();
            cpd_command.points_skill = skill_point_list2.ToArray();
            cpd_command.mode = 0;
            max_step++;
            cpd_command.points_b = skill_point_list2.ToArray();
        }
        else if (process_mode == 11){
            cpd_command.points_a = re_traj.ToArray();
            cpd_command.points_skill = nmpc_marker_list.ToArray();
            cpd_command.mode = 0;
            max_step++;
            cpd_command.points_b = nmpc_marker_list.ToArray();
        }
        else if (process_mode == 40 || process_mode == 42){
            print("Processing...");
            if (process_mode == 40) process_mode = 41;
            else process_mode = 43;            
            robot_count = reproduce_list.Count;
            cpd_command.points_skill = new_skill.ToArray();
            cpd_command.points_a = reproduce_list.ToArray();
            cpd_command.mode = 2;
            max_step++;
            List<RVector7> cut_skill_list = new List<RVector7>();

            int i = 0;
            foreach (var item in new_skill)
            {
                cut_skill_list.Add(item);
                i++;

                if (i >= robot_count)
                {
                    break;
                }
            }

            cpd_command.points_b = cut_skill_list.ToArray();
            

        }
        /*else{
            //robot_point_list.Remove(robot_point_list[0]);
            robot_count = robot_point_list.Count;
            cpd_command.points_a = robot_point_list.ToArray();
            cpd_command.points_skill = skill_point_list.ToArray();
            cpd_command.mode = 0;
            max_step++;
            List<RVector7> cut_skill_list = new List<RVector7>();

            int i = 0;
            foreach (var item in skill_point_list)
            {
                cut_skill_list.Add(item);
                i++;

                if (i >= robot_count)
                {
                    break;
                }
            }

            cpd_command.points_b = cut_skill_list.ToArray();
            
        }*/
        
        
        }
        

        print("DO CPD " + cpd_command.points_a.Length + " " + cpd_command.points_b.Length);

        //MemoryStream ms = new MemoryStream();
        //ms = new MemoryStream();
        //Serializer.Serialize<RRSCPDCommand>(ms, cpd_command);
        //byte[] data = ms.ToArray();

        //movo_ref.publisher_cpd_command.Send(data);
        time_cpd_request_start = DateTime.Now;
        Statics.main_cpd_network.sendMessage(cpd_command);
        
        
    }

    private void OnGUI()
    {
        GUI.skin.label.fontSize = 10;
        GUI.contentColor = Color.black;

        Vector3 scale = new Vector3();

        scale.x = Screen.width / originalWidth;
        scale.y = Screen.height / originalHeight;
        scale.z = 1.0f;

        GUI.matrix = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, scale);

        if (GUI.Button(new Rect(10, 50, 100, 50), "Reset Scene"))
        {
            resetScenario();
        }

        if (GUI.Button(new Rect(150, 50, 100, 50), "Remove Points"))
        {
            removePoints();
        }

        if (skill_point_list.Count == 0)
        {
            GUI.Label(new Rect(10, 150, 300, 50), "Current Step: " + current_step.ToString());
        }
        else
        {
            GUI.Label(new Rect(10, 150, 300, 50), "Current Step: " + current_step.ToString() + " / " + skill_point_list.Count);
        }

        GUI.Label(new Rect(10, 200, 300, 50), "Average Step Time: " + average_teleoperation_two_points_travel_time.ToString());

        if (is_switch_to_skill)
            GUI.Label(new Rect(10, 250, 300, 50), "Network QoS: Disconneted!");
        else
            GUI.Label(new Rect(10, 250, 300, 50), "Network QoS: Connected!");

        string is_rigidt = "Rigid";

        if (is_rigid) is_rigidt = "Rigid"; else is_rigidt = "NonRigid";

        if (operation_mpde == Mode.SharedAutonomyCPD)
            GUI.Label(new Rect(10, 300, 300, 50), "Scenario Mode : " + operation_mpde.ToString() + " " + is_rigidt);
        else
            GUI.Label(new Rect(10, 300, 300, 50), "Scenario Mode : " + operation_mpde.ToString());

        GUI.Label(new Rect(10, 350, 300, 50), "Skill Name : " + skill_file_name);


        if (GUI.Button(new Rect(10, 400, 100, 40), "Nothing"))
        {
            operation_mpde = Mode.Nothing;
        }

        if (GUI.Button(new Rect(10, 450, 100, 40), "Shared"))
        {
            operation_mpde = Mode.SharedAutonomy;
        }

        if (GUI.Button(new Rect(10, 500, 100, 40), "SkillCPD"))
        {
            operation_mpde = Mode.SharedAutonomyCPD;
        }

        if (GUI.Button(new Rect(10, 550, 100, 40), "Play"))
        {
            is_play_scenario = !is_play_scenario;
        }

        //if (GUI.Button(new Rect(10, 600, 100, 40), "Rigid"))
        //{
        //    is_rigid = true;
        //}

        //if (GUI.Button(new Rect(10, 650, 100, 40), "Non-rigid"))
        //{
        //    is_rigid = false;
        //}

        if (GUI.Button(new Rect(10, 810, 100, 40), "Benchmark"))
        {
            bench_state = 1;
        }

        GUI.Label(new Rect(10, 600, 1000, 1000),
        "K: self-iteration plane registration \nJ: cut-frame plane registration \nO: plane detector \nH: point-based plane registeration  \nD: mode selector \nL: Load the skill \nI: skill synthesizer \nX: all-points robot registeration \nC: Add 3D curve/chaos plane\nN: Plane-based robot registration. \nM: extremum-points robot registration \nT: execute the skill");

    }
}
