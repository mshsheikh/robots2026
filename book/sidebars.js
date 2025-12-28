// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Course Content',
      items: [
        'week1-ros2-basics',
        'week2-motion-planning',
        'week3-perception-systems',
        'week4-control-theory',
        'week5-bipedal-locomotion',
        'week6-manipulation-knowledge',
        'week7-humanoid-hardware',
        'week8-ai-integration',
        'week9-human-robot-interaction',
        'week10-ethics-robotics',
        'week11-applications-domains',
        'week12-future-robotics',
        'week13-conclusion',
      ],
    },
    {
      type: 'category',
      label: 'Tools & Resources',
      items: [
        'chat',  // AI Chat Assistant page
      ],
    },
  ],
};

module.exports = sidebars;