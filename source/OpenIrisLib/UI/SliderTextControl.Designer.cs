﻿namespace OpenIris.UI
{
    partial class SliderTextControl
    {
        /// <summary> 
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary> 
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Component Designer generated code

        /// <summary> 
        /// Required method for Designer support - do not modify 
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.textBox = new System.Windows.Forms.TextBox();
            this.trackBar = new System.Windows.Forms.TrackBar();
            this.label = new System.Windows.Forms.Label();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar)).BeginInit();
            this.SuspendLayout();
            // 
            // textBox
            // 
            this.textBox.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.textBox.BackColor = System.Drawing.SystemColors.Control;
            this.textBox.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox.Location = new System.Drawing.Point(804, 18);
            this.textBox.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox.Name = "textBox";
            this.textBox.Size = new System.Drawing.Size(69, 19);
            this.textBox.TabIndex = 47;
            this.textBox.TextChanged += new System.EventHandler(this.textBox_TextChanged);
            // 
            // trackBar
            // 
            this.trackBar.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.trackBar.Location = new System.Drawing.Point(208, 18);
            this.trackBar.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.trackBar.Maximum = 100;
            this.trackBar.Minimum = 100;
            this.trackBar.Name = "trackBar";
            this.trackBar.RightToLeft = System.Windows.Forms.RightToLeft.No;
            this.trackBar.Size = new System.Drawing.Size(586, 69);
            this.trackBar.TabIndex = 46;
            this.trackBar.TickFrequency = 10;
            this.trackBar.TickStyle = System.Windows.Forms.TickStyle.None;
            this.trackBar.Value = 100;
            this.trackBar.Scroll += new System.EventHandler(this.trackBar_Scroll);
            // 
            // label
            // 
            this.label.AutoSize = true;
            this.label.Location = new System.Drawing.Point(4, 23);
            this.label.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label.Name = "label";
            this.label.Size = new System.Drawing.Size(51, 20);
            this.label.TabIndex = 45;
            this.label.Text = "Name";
            // 
            // SliderTextControl
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 20F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.textBox);
            this.Controls.Add(this.trackBar);
            this.Controls.Add(this.label);
            this.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.Name = "SliderTextControl";
            this.Size = new System.Drawing.Size(878, 68);
            ((System.ComponentModel.ISupportInitialize)(this.trackBar)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.TextBox textBox;
        private System.Windows.Forms.TrackBar trackBar;
        private System.Windows.Forms.Label label;
    }
}
