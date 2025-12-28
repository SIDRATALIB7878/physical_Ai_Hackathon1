import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/chat',
    component: ComponentCreator('/chat', '07b'),
    exact: true
  },
  {
    path: '/markdown-page',
    component: ComponentCreator('/markdown-page', '3d7'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'aa8'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'cd1'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '295'),
            routes: [
              {
                path: '/docs/agent_sdk_docs',
                component: ComponentCreator('/docs/agent_sdk_docs', '0ac'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/ai_perception_navigation/isaac_ros_pipelines',
                component: ComponentCreator('/docs/ai_perception_navigation/isaac_ros_pipelines', '4a3'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/ai_perception_navigation/isaac_sim_overview',
                component: ComponentCreator('/docs/ai_perception_navigation/isaac_sim_overview', '73b'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/ai_perception_navigation/overview',
                component: ComponentCreator('/docs/ai_perception_navigation/overview', '79e'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/capstone/autonomous_humanoid',
                component: ComponentCreator('/docs/capstone/autonomous_humanoid', 'acd'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/capstone/overview',
                component: ComponentCreator('/docs/capstone/overview', 'c7c'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/category/ai-perception--navigation',
                component: ComponentCreator('/docs/category/ai-perception--navigation', 'cc0'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/category/capstone',
                component: ComponentCreator('/docs/category/capstone', 'f53'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/category/foundations',
                component: ComponentCreator('/docs/category/foundations', '041'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/category/simulation',
                component: ComponentCreator('/docs/category/simulation', 'd1a'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/category/systems',
                component: ComponentCreator('/docs/category/systems', '548'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/category/tutorial---basics',
                component: ComponentCreator('/docs/category/tutorial---basics', 'e48'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/category/tutorial---extras',
                component: ComponentCreator('/docs/category/tutorial---extras', 'd32'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/category/vision-language-action',
                component: ComponentCreator('/docs/category/vision-language-action', '7ea'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/contributing',
                component: ComponentCreator('/docs/contributing', '7ff'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/foundations/diagrams',
                component: ComponentCreator('/docs/foundations/diagrams', 'a7d'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/foundations/overview',
                component: ComponentCreator('/docs/foundations/overview', '6bc'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/foundations/physical_ai',
                component: ComponentCreator('/docs/foundations/physical_ai', 'a23'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', 'de6'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/pipeline-guide',
                component: ComponentCreator('/docs/pipeline-guide', '1b6'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/simulation/examples/unity_humanoid_project/',
                component: ComponentCreator('/docs/simulation/examples/unity_humanoid_project/', '754'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/simulation/gazebo_basics',
                component: ComponentCreator('/docs/simulation/gazebo_basics', 'bba'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/simulation/overview',
                component: ComponentCreator('/docs/simulation/overview', '931'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/simulation/unity_robotics',
                component: ComponentCreator('/docs/simulation/unity_robotics', '03b'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/systems/overview',
                component: ComponentCreator('/docs/systems/overview', '8cb'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/systems/ros2_intro',
                component: ComponentCreator('/docs/systems/ros2_intro', '92c'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/systems/urdf_control',
                component: ComponentCreator('/docs/systems/urdf_control', '403'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/tutorial-basics/congratulations',
                component: ComponentCreator('/docs/tutorial-basics/congratulations', '99a'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/tutorial-basics/create-a-blog-post',
                component: ComponentCreator('/docs/tutorial-basics/create-a-blog-post', '85c'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/tutorial-basics/create-a-document',
                component: ComponentCreator('/docs/tutorial-basics/create-a-document', 'd2f'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/tutorial-basics/create-a-page',
                component: ComponentCreator('/docs/tutorial-basics/create-a-page', 'dd1'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/tutorial-basics/deploy-your-site',
                component: ComponentCreator('/docs/tutorial-basics/deploy-your-site', '21b'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/tutorial-basics/markdown-features',
                component: ComponentCreator('/docs/tutorial-basics/markdown-features', 'e72'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/tutorial-extras/manage-docs-versions',
                component: ComponentCreator('/docs/tutorial-extras/manage-docs-versions', 'bdf'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/tutorial-extras/translate-your-site',
                component: ComponentCreator('/docs/tutorial-extras/translate-your-site', '6c3'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/vision_language_action/multimodal_interaction',
                component: ComponentCreator('/docs/vision_language_action/multimodal_interaction', '3c8'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/vision_language_action/overview',
                component: ComponentCreator('/docs/vision_language_action/overview', 'bff'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/vision_language_action/whisper_llm',
                component: ComponentCreator('/docs/vision_language_action/whisper_llm', '5a0'),
                exact: true,
                sidebar: "mainSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
