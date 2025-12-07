/**
 * Sidebar configuration for Physical AI & Humanoid Robotics Textbook
 *
 * Structure follows the 6-module layout defined in contracts/sidebar-config-structure.ts:
 * - Module 0: Foundations (3 chapters)
 * - Module 1: ROS 2 (5 chapters)
 * - Module 2: Digital Twin (4 chapters)
 * - Module 3: NVIDIA Isaac (5 chapters)
 * - Module 4: VLA Robotics (3 chapters)
 * - Capstone: Integration Project (1 chapter)
 */

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'Module 0: Physical AI Foundations',
      items: [
        {
          type: 'doc',
          id: 'module-0-foundations/index',
          label: 'Overview',
        },
        {
          type: 'category',
          label: 'Chapter 1: Introduction to Physical AI',
          items: [
            {
              type: 'doc',
              id: 'module-0-foundations/chapter-1/introduction-to-physical-ai',
              label: 'Introduction to Physical AI',
            },
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Robotics Simulation Fundamentals',
          items: [
            {
              type: 'doc',
              id: 'module-0-foundations/chapter-2/robotics-simulation-fundamentals',
              label: 'Robotics Simulation Fundamentals',
            },
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Control Algorithms for Physical AI',
          items: [
            {
              type: 'doc',
              id: 'module-0-foundations/chapter-3/control-algorithms-for-physical-ai',
              label: 'Control Algorithms for Physical AI',
            },
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 for Physical AI',
      items: [
        {
          type: 'doc',
          id: 'module-1-ros2/index',
          label: 'Overview',
        },
        {
          type: 'category',
          label: 'Chapter 1: ROS 2 Architecture',
          items: [
            {
              type: 'doc',
              id: 'module-1-ros2/chapter-1/ros2-architecture',
              label: 'ROS 2 Architecture & Core Concepts',
            },
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Publisher-Subscriber Pattern',
          items: [
            {
              type: 'doc',
              id: 'module-1-ros2/chapter-2/publisher-subscriber',
              label: 'Publisher-Subscriber Pattern',
            },
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Services & Actions',
          items: [
            {
              type: 'doc',
              id: 'module-1-ros2/chapter-3/services-actions',
              label: 'Services & Actions',
            },
          ],
        },
        {
          type: 'category',
          label: 'Chapter 4: TF2 Transforms',
          items: [
            {
              type: 'doc',
              id: 'module-1-ros2/chapter-4/tf-transforms',
              label: 'TF2 Coordinate Transforms',
            },
          ],
        },
        {
          type: 'category',
          label: 'Chapter 5: Robot Control',
          items: [
            {
              type: 'doc',
              id: 'module-1-ros2/chapter-5/robot-control',
              label: 'Robot Control with ros2_control',
            },
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin',
      items: [
        {
          type: 'doc',
          id: 'module-2-digital-twin/index',
          label: 'Overview',
        },
        {
          type: 'category',
          label: 'Chapter 1: Simulation Principles',
          items: [
            {
              type: 'doc',
              id: 'module-2-digital-twin/chapter-1/simulation-principles',
              label: 'Simulation Principles for Physical AI',
            },
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Gazebo Basics',
          items: [
            {
              type: 'doc',
              id: 'module-2-digital-twin/chapter-2/gazebo-basics',
              label: 'Gazebo Robot Simulation',
            },
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Unity Integration',
          items: [
            {
              type: 'doc',
              id: 'module-2-digital-twin/chapter-3/unity-integration',
              label: 'Unity Integration with ROS 2',
            },
          ],
        },
        {
          type: 'category',
          label: 'Chapter 4: Physics Accuracy',
          items: [
            {
              type: 'doc',
              id: 'module-2-digital-twin/chapter-4/physics-accuracy',
              label: 'Physics Accuracy & Validation',
            },
          ],
        },
        // Chapter placeholders will be added as they are created
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac',
      items: [
        {
          type: 'doc',
          id: 'module-3-isaac/index',
          label: 'Overview',
        },
        {
          type: 'category',
          label: 'Chapter 1: Isaac Sim Overview',
          items: [
            {
              type: 'doc',
              id: 'module-3-isaac/chapter-1/isaac-overview',
              label: 'Isaac Sim Overview - GPU-Accelerated Robotics',
            },
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Perception Pipelines',
          items: [
            {
              type: 'doc',
              id: 'module-3-isaac/chapter-2/perception-pipelines',
              label: 'Perception Pipelines in Isaac Sim',
            },
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Reinforcement Learning',
          items: [
            {
              type: 'doc',
              id: 'module-3-isaac/chapter-3/reinforcement-learning',
              label: 'Reinforcement Learning with Isaac Gym',
            },
          ],
        },
        {
          type: 'category',
          label: 'Chapter 4: Sim-to-Real Transfer',
          items: [
            {
              type: 'doc',
              id: 'module-3-isaac/chapter-4/sim-to-real-transfer',
              label: 'Sim-to-Real Transfer with Isaac Sim',
            },
          ],
        },
        {
          type: 'category',
          label: 'Chapter 5: Advanced AI Integration',
          items: [
            {
              type: 'doc',
              id: 'module-3-isaac/chapter-5/advanced-ai-integration',
              label: 'Advanced AI Integration with Isaac Sim',
            },
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA Humanoid Robotics',
      items: [
        {
          type: 'doc',
          id: 'module-4-vla-humanoids/index',
          label: 'Overview',
        },
        {
          type: 'category',
          label: 'Chapter 1: Vision-Language-Action Models',
          items: [
            {
              type: 'doc',
              id: 'module-4-vla-humanoids/chapter-1/vla-overview',
              label: 'Vision-Language-Action Models for Humanoid Robotics',
            },
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Humanoid Control',
          items: [
            {
              type: 'doc',
              id: 'module-4-vla-humanoids/chapter-2/humanoid-control',
              label: 'Humanoid Robot Control with VLA Models',
            },
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: System Integration',
          items: [
            {
              type: 'doc',
              id: 'module-4-vla-humanoids/chapter-3/system-integration',
              label: 'System Integration and Deployment of VLA Humanoid Systems',
            },
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Capstone: Autonomous Humanoid Project',
      items: [
        {
          type: 'doc',
          id: 'capstone/index',
          label: 'Overview',
        },
        {
          type: 'category',
          label: 'Chapter 1: Autonomous Humanoid Project',
          items: [
            {
              type: 'doc',
              id: 'capstone/chapter-1/autonomous-humanoid-project',
              label: 'Autonomous Humanoid Robot Capstone Project',
            },
          ],
        },
      ],
    },
    {
      type: 'doc',
      id: 'instructors/index',
      label: 'Instructor Resources',
    },
    {
      type: 'doc',
      id: 'glossary',
      label: 'Glossary',
    },
  ],
};

module.exports = sidebars;
