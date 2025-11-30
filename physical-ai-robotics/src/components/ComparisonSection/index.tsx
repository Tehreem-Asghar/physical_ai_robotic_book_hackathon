import React from 'react';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

function ComparisonColumn({ title, items, isNewWay }) {
  return (
    <div className={styles.column}>
      <Heading as="h3" className={isNewWay ? styles.newWayTitle : ''}>{title}</Heading>
      <ul>
        {items.map((item, idx) => (
          <li key={idx}>{item}</li>
        ))}
      </ul>
    </div>
  );
}

export default function ComparisonSection() {
  const traditionalItems = [
    "Pre-programmed, rigid behaviors",
    "Relies on detailed, explicit instructions",
    "Struggles with unexpected situations",
    "Operates in highly controlled environments",
    "Sensor data used for simple feedback",
  ];

  const aiNativeItems = [
    "Learns and adapts from experience",
    "Understands high-level, natural language commands",
    "Uses reasoning to solve problems creatively",
    "Thrives in dynamic, human-centric spaces",
    "Fuses multi-modal sensor data for rich understanding",
  ];

  return (
    <section className={styles.comparisonSection}>
      <div className="container">
        <Heading as="h2" className="text--center">The Great Shift: From Automation to Intelligence</Heading>
        <div className={styles.comparisonContainer}>
          <ComparisonColumn title="The Old Way: Traditional Robotics" items={traditionalItems} isNewWay={false} />
          <ComparisonColumn title="The New Way: Physical AI" items={aiNativeItems} isNewWay={true} />
        </div>
      </div>
    </section>
  );
}
