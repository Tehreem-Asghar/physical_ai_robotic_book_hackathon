import React from 'react';
import clsx from 'clsx';
import styles from './AdditionalContent.module.css';

interface FeatureItem {
  title: string;
  description: string;
}

const FeatureList: FeatureItem[] = [
  {
    title: 'Hands-on Learning',
    description: 'Engage with practical examples and guided projects to solidify your understanding of Physical AI and Robotics.',
  },
  {
    title: 'Cutting-Edge Technologies',
    description: 'Explore the latest advancements in ROS2, NVIDIA Isaac Sim, and Visual-Language Models (VLMs) for robotics.',
  },
  {
    title: 'Community & Support',
    description: 'Join a vibrant community of learners and experts, share insights, and get support on your robotics journey.',
  },
];

function Feature({ title, description }: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        {/* You can add an SVG or icon here if desired */}
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function AdditionalContent(): JSX.Element {
  return (
    <section className={styles.additionalContent}>
      <div className="container">
        <div className="row">
          <div className={clsx('col col--12 text--center')}>
            <h2>Why Learn Physical AI & Robotics?</h2>
            <p>Unlock the future with skills in intelligent autonomous systems.</p>
          </div>
        </div>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
