classdef symbolicMR < handle
    %symbolicMR the kinematics of MR model but in symbolic math form
    %   This class shall hold the information required to construct a
    %   kinematic model of MR using the method of Kelly and Seegmiller.
    
    properties
        % these are physical parameters of the MR model in m
        % lateral width from CG
        w_ = 0.1;
        % longitudinal distance from CG
        l_ = 0.2;
        % vertical distance from CG
        h_ = 0.1;
        % wheel radius
        r_ = 0.05;
        % number wheels
        nWheels_ = 4;
        
        % this kinematic table relates each frame to the other frames
        table_ = [];
                
        % current vehicle state
        q_ = sym(zeros(10, 1));
        q_dot_ = sym(zeros(10, 1));
        % contact frame slip constraints
        vc_ = sym(zeros(12, 1));
        % current world transform list
        tList_;
    end
    
    methods
        function obj = symbolicMR(w, l, h, r, initial_state)
            %symbolicMR Construct an instance of this class
            %   assign the w, l, h, and also intialize nodes to construct
            %   the tree.
            %
            if nargin ~= 0
                obj.w_ = w;
                obj.l_ = l;
                obj.h_ = h;
                obj.r_ = r;
                obj.q_ = initial_state;
            end
            
            % initialize the tree nodes
            world = coordinateFrame(0, 0, eye(4));
            body = coordinateFrame(0, 0, eye(4));
            fl = coordinateFrame(0, 0, eye(4));
            fr = coordinateFrame(0, 0, eye(4));
            bl = coordinateFrame(0, 0, eye(4));
            br = coordinateFrame(0, 0, eye(4));
            % contact frames are all -r in z-axis from their parent
            flc = coordinateFrame(fl, [], 0);
            frc = coordinateFrame(fr, [], 0);
            blc = coordinateFrame(bl, [], 0);
            brc = coordinateFrame(br, [], 0);
            
            % i'll try using a table structure
            % all joints are RY joints. 
            obj.table_.i = [0 1 2 3 4];
            % current frame
            obj.table_.frame = [body, fl, fr, bl, br];
            % parent frame
            obj.table_.parent = [world, body, body, body, body];
            % actuated?
            obj.table_.act = [0, 1, 1, 1, 1];
            % pose to parent
            obj.table_.x = [0, obj.l_, obj.l_, -obj.l_, -obj.l_];
            obj.table_.y = [0, obj.w_, -obj.w_, obj.w_, -obj.w_];
            obj.table_.z = [0, -obj.h_, -obj.h_, -obj.h_, -obj.h_];
            
            
        end
        
        
        function transforms = constructTreeFromTable(obj)
            %constructTree builds the tree from the table given 
            % returns - transforms:
            %   outputs transform from world to frame for all frames
            %   in the order:
            %       1) world->body, 
            %       2) world->fl, 
            %       3) world->fr, 
            %       4) world->bl,
            %       5) world->br, 
            %       6) world->flc, 7) world->frc, 8) world->blc, 9) world->brc
            
            % get world to base from current state
            ori = obj.q_(1:3);
            pos = obj.q_(4:6);
            t_w2b = obj.transformGenerate(0, pos);
            % now to apply any rotations
            % TODO: check the order of the euler angles
            % x axis rotation
            alpha = ori(1);
            Rx = [1     0           0       0;
                  0 cos(alpha) -sin(alpha)  0;
                  0 sin(alpha)  cos(alpha)  0;
                  0     0           0       1];
            % y axis rotation
            beta = ori(2);
            Ry = [cos(beta)  0 sin(beta)  0;
                     0       1     0      0;
                 -sin(beta)  0 cos(beta)  0;
                     0       0     0      1];
            % z axis rotation
            gamma = ori(3);
            Rz = [cos(gamma) -sin(gamma) 0 0;
                  sin(gamma)  cos(gamma) 0 0;
                     0           0       1 0;
                     0           0       0 1];
            euler = Rz*Ry*Rx;
            t_w2b = t_w2b * euler;

            
            joints = obj.q_(6:10);
            
            transforms(:, :, 1) = t_w2b;
            for i = 2:length(obj.table_.i)
                % get translation relative to parent
                d(1) = obj.table_.x(i);
                d(2) = obj.table_.y(i);
                d(3) = obj.table_.z(i);
                
                % get transform relative to parent
                t_b2i = obj.transformGenerate(joints(i), d);
                
                % world to current 
                t_w2i = t_w2b * t_b2i;
                
                % put world to current in array
                transforms(:, :, i) = t_w2i;
                
                % put the contact frame at the end
                t_b2iPRIME = t_b2i;
                t_b2iPRIME(1:3, 1:3) = eye(3);
                t_p2c = obj.transformGenerate(0, [0;0;-obj.r_]);
                transforms(:, :, i + 4) = t_w2b * t_b2iPRIME * t_p2c;
                                
            end
       
        end
        
        function T = transformGenerate(obj, theta, d)
            %transformGenerate constructs the transform matrix for a joint
            %{
            args:
               theta -  joint value
               d - pose relative to parent frame in translation only
            returns:
               T - homogenous transformation matrix
             %}
            
%           since everything is an RY joint, this is the y-axis
%           rotation matrix
           
            R = [cos(theta)  0 sin(theta);
                     0       1    0      ;
                 -sin(theta) 0 cos(theta)];
             % assemble the transform
            T(1:3, 4) = d;
            T(1:3, 1:3) = R;
            T(4, 1:end) = [0 0 0 1];
        
        end
        
        function J = jacobianGenerate(obj, tList)
            %jacobianGenerate constructs the wheel jacobian for all wheels
            % args:
            %    t_list - list of transforms from world to each frame
            % returns:
            %    J - stacked wheel Jacobian for calculating vehicle velocity
            %    propogation
            
            % collect information from body frame
            bodyT = tList(:, :, 1);
            R_w2b = bodyT(1:3, 1:3);
            d_w2b = bodyT(1:3, 4);
%             % preallocate full stack (4 wheels)
%             aWheelStack = zeros(3*obj.nWheels_, 10);
            J = [];
            for i = 2:5
                % i is wheel joint frame not wheel contact frame
                wheelT = tList(:, :, i);
                contactT = tList(:, :, i+4);
                aWheel = sym(zeros(3, 10));

                % revolute joints insert this familiar cross product
                % rotation matrix world to wheel
                R_w2i = wheelT(1:3, 1:3);
                R_w2c = contactT(1:3, 1:3);
                axis = R_w2i(:, 2);
                % translation vectors
                d_w2c = contactT(1:3, 4);
                d_w2i = wheelT(1:3, 4);
                d = d_w2c - d_w2i;
                % insert column for joint variable
                % NOTE: the negative sign is due to traction...this constraint doesn't
                % necessarily have to be enforced here
                aWheel(:, i + 5) = cross(axis, d);

                % insert body information orientation then translation
                aWheel(:, 1:3) = skew(d_w2c - d_w2b)'*R_w2b;
                aWheel(:, 4:6) = R_w2b;
                aWheel = R_w2c' * aWheel;
                % insert wheel into stack
                J = [J; aWheel];
                
            end
            disp(simplify(J))
        end
        
        function q = navigationLoop(obj, q_dot, dt)
            %navigationLoop goes through a 'forward kinematics' pass of the
            % robot, calculating body velocity from desired joint
            % velocities
            %   args:
            %       q_dot - desired joint velocities within full state
            %       dt - time step
            %   returns:
            %       q - updated state
            
            obj.q_dot_ = q_dot;
            obj.tList_ = obj.constructTreeFromTable;
            % solve for unknown variables
            obj.motionPrediction('nav');
            
            % integration to update state
            V = obj.cartesian2SpatialVelocity;
            obj.q_ = obj.q_ + (V*obj.q_dot_) * dt;
            q = obj.q_;
            
        end
        
        function q = actuationLoop(obj, q_dot, dt)
            %actuationLoop goes through an 'inverse' pass of the robot,
            % calculating wheel velocities to achieve desired body frame
            % velocity
            %   args:
            %       q_dot - desired body velocities within full state
            %       dt - time step
            
            obj.q_dot_ = q_dot;
            obj.tList_ = obj.constructTreeFromTable;
            % solve for unknowns
            obj.motionPrediction('act');
            
            % integration
            V = obj.cartesian2SpatialVelocity;
            obj.q_ = obj.q_ + (V*obj.q_dot_) * dt;
            q = obj.q_;
        end
        
        function output_q = motionPrediction(obj, direction)
            %{
            motionPrediction performs the calculation to predict motion
            which is based off of kinematic constraints
            
            args:
                direction - navigation/actuation kinematics
                            a) navigation kinematics goes from 
                               actuated joint rates to body velocity
                            b) actuation kinematics goes from desired body
                               velocity to actuated joint rates
            returns:
                output_q - the full state with the unknown variables solved
                for
                
            %}
            % slice up the state vector
            body_velocity = obj.q_dot_(1:6);
            joint_rate = obj.q_dot_(7:10);

            % calculate wheel jacobian
            J = obj.jacobianGenerate(obj.tList_);
            % slice it up
            J_v = J(:, 1:6);
            J_r = J(:, 7:10);

            if strcmp('nav', direction)
                % slice up the jacobian and state to calculate body
                % velocity from joint rates
                body_velocity = pinv(J_v)*(obj.vc_ - J_r*joint_rate);
            
            elseif strcmp('act', direction)
                joint_rate = pinv(J_r)*(obj.vc_ - J_v*body_velocity);
%                 disp(obj.vc_ - J_v*body_velocity)
%                 disp(joint_rate)
            
            else
                disp("[direction]: Choose between navigation ('nav') and acutation kinematics ('act')")
                
            end
            
            obj.q_dot_(1:6) = body_velocity;
            obj.q_dot_(7:10) = joint_rate;
            output_q = obj.q_dot_;
        
              
        end
        
        function V = cartesian2SpatialVelocity(obj)
            %cartesian2SpatialVelocity transforms q_dot into q_dot prime
            T_w2b = obj.tList_(:, :, 1);
            R_w2b = T_w2b(1:3, 1:3);
            I = eye(4);
            omega = obj.computeOmega(obj.q_(1:3));
            V = blkdiag(omega, R_w2b, I);
        end
        
        function omega = computeOmega(obj, ori)
            %calculates transform between spatial velocity and time derivative of pose.
            phi = ori(1);
            theta = ori(2);
            psi = ori(3);
            omega = [1  sin(phi)*tan(theta)  cos(phi)*tan(theta);
                     0       cos(phi)             -sin(phi)     ;
                     0  sin(phi)/cos(theta)  cos(phi)/cos(theta)];
        end

    end
end

