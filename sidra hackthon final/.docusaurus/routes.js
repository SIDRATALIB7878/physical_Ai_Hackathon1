import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '3d1'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '1f9'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '665'),
            routes: [
              {
                path: '/docs/agent_sdk_docs',
                component: ComponentCreator('/docs/agent_sdk_docs', 'bb8'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/ai_perception_navigation/isaac_ros_pipelines',
                component: ComponentCreator('/docs/ai_perception_navigation/isaac_ros_pipelines', '9a3'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/ai_perception_navigation/isaac_sim_overview',
                component: ComponentCreator('/docs/ai_perception_navigation/isaac_sim_overview', '3f4'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/ai_perception_navigation/overview',
                component: ComponentCreator('/docs/ai_perception_navigation/overview', 'eff'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/capstone/autonomous_humanoid',
                component: ComponentCreator('/docs/capstone/autonomous_humanoid', '6c8'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/capstone/overview',
                component: ComponentCreator('/docs/capstone/overview', 'b54'),
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
                component: ComponentCreator('/docs/contributing', 'b08'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/foundations/diagrams',
                component: ComponentCreator('/docs/foundations/diagrams', 'f60'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/foundations/overview',
                component: ComponentCreator('/docs/foundations/overview', '0f3'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/foundations/physical_ai',
                component: ComponentCreator('/docs/foundations/physical_ai', '297'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '1d7'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/pipeline-guide',
                component: ComponentCreator('/docs/pipeline-guide', 'e7e'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/simulation/examples/unity_humanoid_project/',
                component: ComponentCreator('/docs/simulation/examples/unity_humanoid_project/', 'aaf'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/simulation/gazebo_basics',
                component: ComponentCreator('/docs/simulation/gazebo_basics', '86b'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/simulation/overview',
                component: ComponentCreator('/docs/simulation/overview', '920'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/simulation/unity_robotics',
                component: ComponentCreator('/docs/simulation/unity_robotics', '864'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/systems/overview',
                component: ComponentCreator('/docs/systems/overview', 'f76'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/systems/ros2_intro',
                component: ComponentCreator('/docs/systems/ros2_intro', 'eae'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/systems/urdf_control',
                component: ComponentCreator('/docs/systems/urdf_control', '57e'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/tutorial-basics/congratulations',
                component: ComponentCreator('/docs/tutorial-basics/congratulations', 'e9e'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/tutorial-basics/create-a-blog-post',
                component: ComponentCreator('/docs/tutorial-basics/create-a-blog-post', 'd61'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/tutorial-basics/create-a-document',
                component: ComponentCreator('/docs/tutorial-basics/create-a-document', '590'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/tutorial-basics/create-a-page',
                component: ComponentCreator('/docs/tutorial-basics/create-a-page', 'afd'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/tutorial-basics/deploy-your-site',
                component: ComponentCreator('/docs/tutorial-basics/deploy-your-site', '8b9'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/tutorial-basics/markdown-features',
                component: ComponentCreator('/docs/tutorial-basics/markdown-features', 'acf'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/tutorial-extras/manage-docs-versions',
                component: ComponentCreator('/docs/tutorial-extras/manage-docs-versions', '414'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/tutorial-extras/translate-your-site',
                component: ComponentCreator('/docs/tutorial-extras/translate-your-site', 'b1f'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/vision_language_action/multimodal_interaction',
                component: ComponentCreator('/docs/vision_language_action/multimodal_interaction', '1c2'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/vision_language_action/overview',
                component: ComponentCreator('/docs/vision_language_action/overview', 'a80'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/docs/vision_language_action/whisper_llm',
                component: ComponentCreator('/docs/vision_language_action/whisper_llm', '9be'),
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
    path: '*',
    component: ComponentCreator('*'),
  },
];
